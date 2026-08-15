"""robot 操作 GUI ノード (robot_web_gui)。

dummy_input (端末) のブラウザ版。HTTP で受けた操作を state_command (1 byte)
として robot_control_manager (RCM) に送り、state_status で現在状態を表示する。
motor_commands を直接送る web_controller と違い、必ず RCM の状態機械を経由する。

  ブラウザ(PC) ──POST /cmd──▶ robot_web_gui (robot) ──state_command──▶ RCM
                                     ▲───────────── state_status ──────┘

状態表示は必ず受信した state_status 駆動 (fault や watchdog で非同期に OFF に
落ちるため、自分が送ったコマンドから状態を推測しない)。

RUN ゲート (READY 補間完了まで RUN 不可):
  RCM は補間進捗を publish しない (state_status は READY のまま) ので、
  STOP→READY 遷移を観測した時刻から interpolation_time (config) + マージンを
  経過測定する。RUN は受理されるまで毎 tick 再送 (RCM は未完了 RUN を
  ログのみで黙って拒否するため。sysid_controller と同じリトライ方式)。

環境変数:
  ROBOT_WEB_GUI_PORT   HTTP ポート (既定 8766。8765=web_monitor, 8770=web_controller)
  ROBOT_CONFIG         robot_config JSON (相対パスはリポジトリルート基準)
"""

import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pyarrow as pa
from dora import Node

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.enum_def import State, StateCommand  # noqa: E402
from lib.sensor_data_format import unpack_imu_data  # noqa: E402
from lib import robot_config  # noqa: E402

PORT = int(os.environ.get("ROBOT_WEB_GUI_PORT", "8766"))
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(SCRIPT_DIR, "robot_web_gui.html")
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))

# READY 補間完了判定のマージン [s] (interp は motor_status tick 駆動 ≒ 壁時計)
READY_MARGIN_S = 0.3
# RUN リトライの上限 [s] (超えたらエラー表示)
RUN_RETRY_TIMEOUT_S = 3.0
# state_status 途絶判定 [s]
LINK_TIMEOUT_S = 0.5

CMD_NAMES = {
    "servo_on": StateCommand.SERVO_ON,
    "servo_off": StateCommand.SERVO_OFF,
    "stop": StateCommand.STOP,
    "ready": StateCommand.READY,
    "run": StateCommand.RUN,
    "reset": StateCommand.INIT_POSITION_RESET,
}


def _load_config():
    path = os.environ.get("ROBOT_CONFIG", "")
    if path:
        if not os.path.isabs(path):
            path = os.path.join(PROJECT_ROOT, path)
        try:
            return robot_config.load_from_file(path)
        except (OSError, KeyError, ValueError, json.JSONDecodeError) as e:
            print(f"[robot_web_gui] ROBOT_CONFIG 読込失敗: {e}")
    return None


CFG = _load_config()
ROBOT_NAME = CFG.robot_name if CFG else "(no config)"
INTERP_TIME = CFG.interpolation_time if CFG else 3.0


class GuiState:
    """HTTP スレッドと node ループで共有する状態。lock で保護。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = None            # 最新 state_status (int) / None = 未受信
        self.state_t = 0.0           # 受信時刻 (monotonic)
        self.ready_start = None      # STOP→READY を観測した時刻
        self.run_pending_since = None  # RUN リトライ開始時刻 / None = 非リトライ中
        self.last_cmd = "(none)"
        self.error = ""
        self.cmd_q = queue.Queue(maxsize=16)  # 要素 = StateCommand
        self.imu = None                # 最新 ImuData (namedtuple) / None = 未受信

    # --- node ループ側 ---

    def update_state(self, new_state: int):
        with self.lock:
            old = self.state
            self.state = new_state
            self.state_t = time.monotonic()
            if new_state == State.READY:
                if old != State.READY and old is not None:
                    self.ready_start = time.monotonic()  # STOP→READY 遷移
            else:
                self.ready_start = None
                if new_state == State.RUN:
                    self.run_pending_since = None  # RUN 受理された
                elif self.run_pending_since is not None:
                    # READY から外れた → リトライ中止
                    self.run_pending_since = None
                    self.error = "RUN 中断: READY から離脱"

    def ready_progress_locked(self) -> float:
        if self.state != State.READY or self.ready_start is None:
            return 1.0 if self.state == State.RUN else 0.0
        return min(1.0, (time.monotonic() - self.ready_start)
                   / (INTERP_TIME + READY_MARGIN_S))

    # --- HTTP 側 ---

    def snapshot(self) -> dict:
        with self.lock:
            now = time.monotonic()
            imu = None
            if self.imu is not None:
                imu = {
                    "roll": round(self.imu.roll, 4),
                    "pitch": round(self.imu.pitch, 4),
                    "yaw": round(self.imu.yaw, 4),
                    "gx": round(self.imu.gx, 4),
                    "gy": round(self.imu.gy, 4),
                    "gz": round(self.imu.gz, 4),
                }
            return {
                "robot_name": ROBOT_NAME,
                "state": State(self.state).name if self.state is not None else "----",
                "ready_progress": round(self.ready_progress_locked(), 3),
                "run_pending": self.run_pending_since is not None,
                "link_ok": self.state is not None
                           and (now - self.state_t) < LINK_TIMEOUT_S,
                "interpolation_time": INTERP_TIME,
                "last_cmd": self.last_cmd,
                "error": self.error,
                "imu": imu,
            }

    def request(self, name: str):
        """コマンド要求。RCM の遷移ガードをミラーして検証 (RCM 側でも再検証される)。
        受理したらキューに積む。ValueError = 拒否 (理由付き)。"""
        cmd = CMD_NAMES.get(name)
        if cmd is None:
            raise ValueError(f"unknown cmd: {name}")
        with self.lock:
            st = self.state
            if st is None:
                raise ValueError("state_status 未受信 (dataflow 停止中?)")
            if cmd == StateCommand.SERVO_ON and st != State.OFF:
                raise ValueError("SERVO_ON は OFF 状態でのみ可")
            if cmd == StateCommand.INIT_POSITION_RESET and st != State.OFF:
                raise ValueError("基準姿勢セットは OFF 状態でのみ可")
            if cmd == StateCommand.READY and st != State.STOP:
                raise ValueError("READY は STOP 状態でのみ可")
            if cmd == StateCommand.STOP and st == State.OFF:
                raise ValueError("STOP は OFF 状態では不可")
            if cmd == StateCommand.RUN:
                if st != State.READY:
                    raise ValueError("RUN は READY 状態でのみ可")
                if self.ready_progress_locked() < 1.0:
                    raise ValueError("READY 補間が未完了")
                # RUN はリトライ方式 (node ループが受理まで再送)
                self.run_pending_since = time.monotonic()
                self.error = ""
                self.last_cmd = "RUN"
                return
            self.error = ""
            self.last_cmd = name.upper()
            self.cmd_q.put_nowait(cmd)


GUI = GuiState()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_html()
        elif self.path == "/status":
            self._json(200, GUI.snapshot())
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/cmd":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length) or b"{}")
            GUI.request(body.get("cmd", ""))
        except (ValueError, TypeError, json.JSONDecodeError, queue.Full) as e:
            self._json(409, {"ok": False, "error": str(e)})
            return
        self._json(200, {"ok": True})

    def _serve_html(self):
        try:
            with open(HTML_PATH, "rb") as fp:
                body = fp.read()
        except OSError:
            self.send_error(500, "robot_web_gui.html not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def serve_http():
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    httpd.daemon_threads = True
    print(f"robot_web_gui: http://0.0.0.0:{PORT}/")
    httpd.serve_forever()


def send_cmd(node, cmd: StateCommand):
    node.send_output("state_command", pa.array([int(cmd)], type=pa.uint8()))


def main():
    threading.Thread(target=serve_http, daemon=True).start()
    node = Node("robot_web_gui")
    print(f"[robot_web_gui] ready: {ROBOT_NAME} (interp {INTERP_TIME}s)")
    for event in node:
        if event["type"] != "INPUT":
            continue
        if event["id"] == "state_status":
            raw = event["value"].to_pylist()
            if raw:
                GUI.update_state(raw[0])
        elif event["id"] == "imu_data":
            # 最新値を latch するだけ (表示は /status ポーリング 5Hz 側で間引き)
            raw = bytes(event["value"].to_pylist())
            if len(raw) >= 112:
                with GUI.lock:
                    GUI.imu = unpack_imu_data(raw)
        elif event["id"] == "tick":
            # 通常コマンドを drain (送信は必ずこのメインスレッド)
            while True:
                try:
                    cmd = GUI.cmd_q.get_nowait()
                except queue.Empty:
                    break
                send_cmd(node, cmd)
                print(f"[robot_web_gui] sent: {StateCommand(cmd).name}")
            # RUN リトライ: 受理 (state==RUN) まで毎 tick 再送。
            # RCM は補間未完了の RUN をログのみで黙って拒否するため。
            with GUI.lock:
                pending = GUI.run_pending_since
            if pending is not None:
                if time.monotonic() - pending > RUN_RETRY_TIMEOUT_S:
                    with GUI.lock:
                        GUI.run_pending_since = None
                        GUI.error = "RUN タイムアウト (RCM が受理せず)"
                    print("[robot_web_gui] RUN retry timeout")
                else:
                    send_cmd(node, StateCommand.RUN)


if __name__ == "__main__":
    main()
