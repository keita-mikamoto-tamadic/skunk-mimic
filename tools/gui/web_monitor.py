"""web モータ監視 GUI ―― dataflow 非依存のトピックブリッジ。

設計の肝:
  motor_status は foctive でも mimic でもどんな相手でも同じ
  `device_control_manager/motor_status`。だからこのモニタは特定 dataflow に
  紐づかない。`dora topic echo` (dataflow 非依存のトピック購読) で生バイトを
  受け、AxisAct にデコードして、ブラウザへ JSON を SSE 配信するだけ。

  [実行中 dataflow を自動発見] ─ dora list
            │
            ▼
  dora topic echo -d <df> device_control_manager/motor_status --format json
            │  {"data":[<AxisAct 48B>...], ...}  ← dora が envelope を解いて生バイトを渡す
            ▼
  web_monitor.py ── AxisAct デコード(lib.data_format) ──HTTP/SSE──> web_monitor.html

特徴:
  * dora node add も deploy も daemon 配置も不要。dataflow に一切属さない。
  * `dora topic echo` は coordinator に WS で繋ぐので PC からでも robot からでも動く。
  * 前提は dataflow 側の `_unstable_debug.enable_debug_inspection: true` だけ
    (トピック inspection 一般に必要。monitor 固有の依存ではない)。
  * dataflow が止まる/別の相手(mimic 等)に変わっても、自動で再発見して追従する。

依存: Python stdlib のみ (lib.data_format は struct だけ)。`dora` CLI が要る。

環境変数:
  DORA_COORDINATOR_ADDR  coordinator の IP (dora CLI が読む。既定 127.0.0.1)
  WEB_MONITOR_PORT       HTTP ポート (既定 8765)
  DORA_TOPIC             購読トピック (既定 device_control_manager/motor_status)
  DORA_DATAFLOW          固定したい時の dataflow 名/UUID (既定: 実行中を自動発見)
  DORA_BIN               dora 実行ファイルパス (既定: PATH か ~/dora/target/release/dora)
  ROBOT_CONFIG           軸名/CAN ID 表示用 (任意)
"""

import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --- lib (axis_data.json 正本から自動生成) を import パスに追加 -----------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src", "python"))
from lib.data_format import AXIS_ACT_SIZE, unpack_axis_act  # noqa: E402

PORT = int(os.environ.get("WEB_MONITOR_PORT", "8765"))
TOPIC = os.environ.get("DORA_TOPIC", "device_control_manager/motor_status")
FIXED_DATAFLOW = os.environ.get("DORA_DATAFLOW")  # None なら自動発見
PUB_INTERVAL_S = 0.033  # ~30Hz。高 tick(1kHz)でもブラウザへは間引いて送る


def find_dora():
    cand = os.environ.get("DORA_BIN") or shutil.which("dora") \
        or os.path.expanduser("~/dora/target/release/dora")
    if not (cand and os.path.exists(cand)):
        sys.exit("web_monitor: `dora` CLI が見つかりません。DORA_BIN で指定してください。")
    return cand


DORA = find_dora()

# config (任意): 軸名/CAN ID を JSON に載せるためだけに使う。失敗しても続行。
AXIS_META = []  # [(name, can_id), ...]
_env_config = os.environ.get("ROBOT_CONFIG")
if _env_config:
    try:
        from lib import robot_config  # noqa: E402

        cfg_path = _env_config if os.path.isabs(_env_config) \
            else os.path.join(PROJECT_ROOT, _env_config)
        _cfg = robot_config.load_from_file(cfg_path)
        AXIS_META = [(ax.name, ax.device_id) for ax in _cfg.axes]
        print(f"web_monitor: loaded config {_cfg.robot_name} ({len(AXIS_META)} axes)")
    except Exception as e:
        print(f"web_monitor: config 読み込み失敗 (無視して続行): {e}")


class Broadcaster:
    """SSE 購読者へ最新スナップショットを配る。スレッド安全。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._subs = set()      # set[queue.Queue]
        self._latest = None     # 直近 JSON (新規接続へ即送信)

    def subscribe(self):
        q = queue.Queue(maxsize=4)
        with self._lock:
            self._subs.add(q)
            latest = self._latest
        if latest is not None:
            q.put_nowait(latest)
        return q

    def unsubscribe(self, q):
        with self._lock:
            self._subs.discard(q)

    def publish(self, payload: str):
        with self._lock:
            self._latest = payload
            subs = list(self._subs)
        for q in subs:
            try:
                q.put_nowait(payload)
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait(payload)
                except queue.Empty:
                    pass


BROADCAST = Broadcaster()


# パス -> (ファイル名, Content-Type)。SCRIPT_DIR 内のみ配信 (traversal 防止)。
STATIC = {
    "/": ("web_monitor.html", "text/html; charset=utf-8"),
    "/index.html": ("web_monitor.html", "text/html; charset=utf-8"),
    "/uPlot.iife.min.js": ("uPlot.iife.min.js", "application/javascript"),
    "/uPlot.min.css": ("uPlot.min.css", "text/css"),
}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self._serve_stream()
        elif self.path in STATIC:
            fname, ctype = STATIC[self.path]
            self._serve_static(fname, ctype)
        else:
            self.send_error(404)

    def _serve_static(self, fname, ctype):
        try:
            with open(os.path.join(SCRIPT_DIR, fname), "rb") as f:
                body = f.read()
        except OSError:
            self.send_error(500, f"{fname} not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = BROADCAST.subscribe()
        try:
            while True:
                payload = q.get()
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            BROADCAST.unsubscribe(q)


def serve_http():
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    httpd.daemon_threads = True
    print(f"web_monitor: http://0.0.0.0:{PORT}/  (SSE: /stream)")
    httpd.serve_forever()


def decode_axes(raw: bytes):
    count = len(raw) // AXIS_ACT_SIZE
    axes = []
    for i in range(count):
        a = unpack_axis_act(raw, i * AXIS_ACT_SIZE)
        axis = {
            "position": a.position, "velocity": a.velocity, "torque": a.torque,
            "cur_d": a.cur_d, "cur_q": a.cur_q, "fault": a.fault,
        }
        if i < len(AXIS_META):
            axis["name"], axis["can_id"] = AXIS_META[i]
        axes.append(axis)
    return axes


def discover_dataflow():
    """実行中の dataflow 名を返す (foctive/mimic 問わず最初の Running)。無ければ None。"""
    if FIXED_DATAFLOW:
        return FIXED_DATAFLOW
    try:
        out = subprocess.run([DORA, "list", "--format", "json"],
                             capture_output=True, text=True, timeout=10)
    except (subprocess.SubprocessError, OSError):
        return None
    for line in out.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if obj.get("status") == "Running":
            return obj.get("name") or obj.get("uuid")
    return None


def stream_loop():
    """dataflow を発見し dora topic echo を回し続ける。落ちたら再発見して追従。"""
    last_pub = 0.0
    while True:
        df = discover_dataflow()
        if not df:
            print("web_monitor: 実行中の dataflow が見つかりません。再試行...")
            time.sleep(2)
            continue
        print(f"web_monitor: dataflow `{df}` の {TOPIC} を購読")
        proc = subprocess.Popen(
            [DORA, "topic", "echo", "-d", df, TOPIC, "--format", "json"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        try:
            for line in proc.stdout:
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                data = obj.get("data")
                if not data:
                    continue
                raw = bytes(data)
                if len(raw) < AXIS_ACT_SIZE:
                    continue
                now = time.monotonic()
                if now - last_pub >= PUB_INTERVAL_S:  # ~30Hz に間引き
                    # x 軸用の時刻 [秒]。dora timestamp は unix ms。無ければ wall clock。
                    ts = obj.get("timestamp")
                    t = ts / 1000.0 if isinstance(ts, (int, float)) else time.time()
                    BROADCAST.publish(json.dumps({"t": t, "axes": decode_axes(raw)}))
                    last_pub = now
        finally:
            proc.terminate()
        # echo が終了 (dataflow 停止/再起動) → 少し待って再発見
        print("web_monitor: topic echo 終了。再発見します...")
        time.sleep(1)


def main():
    threading.Thread(target=serve_http, daemon=True).start()
    stream_loop()


if __name__ == "__main__":
    main()
