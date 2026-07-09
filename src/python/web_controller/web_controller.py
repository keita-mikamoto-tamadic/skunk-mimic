"""FOCTIVE Web 操作ノード (web_controller)。

foctive_controller(端末版) のブラウザ版。stdin の代わりに HTTP で指令を受け、
AxisRef(motor_commands) を送る本物の dora ノード ―― バイナリ指令を正しく
出せるのはノードだけ (`dora topic pub` は JSON テキストしか送れない)。

  ブラウザ(PC) ──POST /cmd──▶ web_controller (PC daemon) ──motor_commands──▶ device_control_manager (robot)

分散構成: web_controller は dataflow で PC 側 daemon に deploy される
(dataflow_foctive_web_control.yaml の _unstable_deploy.machine: pc)。
動的ノードは localhost daemon にアタッチするので、必ず PC 上で起動すること。

送信はスレッド安全のため「HTTP スレッドはキューに積む / node ループ(tick)で
まとめて send_output」する。送信は node メインスレッドからのみ。

環境変数:
  WEB_CONTROLLER_PORT   HTTP ポート (既定 8770)
"""

import json
import os
import queue
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pyarrow as pa
from dora import Node

# lib (axis_data.json 正本から自動生成) を import パスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.data_format import AxisRef, pack_axis_ref  # noqa: E402

PORT = int(os.environ.get("WEB_CONTROLLER_PORT", "8770"))
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(SCRIPT_DIR, "web_controller.html")

# MotorState (enum_def.hpp / foctive_controller.py と一致)
MOTOR_OFF = 0
MOTOR_POSITION = 2          # 位置(インピーダンス)
MOTOR_VELOCITY = 3          # 速度(インピーダンス)
MOTOR_CURRENT = 6
MOTOR_VOLTAGE = 7
MOTOR_POSITION_PD = 8       # インピーダンス (pos/vel/torq)
MOTOR_CASCADE_POS_PID = 9   # FOCTIVE ネイティブ位置カスケードPID
MOTOR_CASCADE_VEL_PID = 10  # FOCTIVE ネイティブ速度カスケードPID

# HTTP スレッド → node ループへ渡す送信キュー。要素 = (AxisRef, 説明文字列)
CMD_Q = queue.Queue(maxsize=64)
LAST_SENT = {"desc": "(none)"}  # /status 用


def build_ref(typ, p):
    """指令 type + パラメータ dict から (AxisRef, desc) を作る。

    マッピングは foctive_controller.py と一致 (impedance/cascade 対応)。
    p は {名前: 数値} の dict。欠けたキーは default を使う。
    """
    p = p or {}

    def n(key, default=0.0):
        v = p.get(key)
        return float(v) if isinstance(v, (int, float)) else float(default)

    if typ in ("off", "f"):
        return AxisRef(motor_state=MOTOR_OFF), "SERVO OFF"
    if typ == "v":    # 電圧: ref_val=volt_d, ref_val_1=volt_q, ref_val_2=vir_ang_freq
        d, q, fr = n("volt_d"), n("volt_q"), n("vir_ang_freq")
        return (AxisRef(motor_state=MOTOR_VOLTAGE, ref_val=d, ref_val_1=q, ref_val_2=fr,
                        kp_scale=1.0, kv_scale=1.0),
                f"VOLTAGE d={d} q={q} freq={fr}")
    if typ == "c":    # 電流: ref_val=cur_d, ref_val_1=cur_q
        d, q = n("cur_d"), n("cur_q")
        return (AxisRef(motor_state=MOTOR_CURRENT, ref_val=d, ref_val_1=q,
                        kp_scale=1.0, kv_scale=1.0),
                f"CURRENT d={d} q={q}")
    if typ == "vel":  # 速度(インピーダンス): ref_val=vel, kp/kd/accel_limit
        vel, kp, kd, ac = n("vel"), n("kp", 1.0), n("kd", 1.0), n("accel", 0.0)
        return (AxisRef(motor_state=MOTOR_VELOCITY, ref_val=vel, accel_limit=ac,
                        kp_scale=kp, kv_scale=kd),
                f"VELOCITY vel={vel} kp={kp} kd={kd} accel={ac}")
    if typ == "p":    # 位置(インピーダンス): ref_val=pos, kp/kd/accel_limit
        pos, kp, kd, ac = n("pos"), n("kp", 1.0), n("kd", 1.0), n("accel", 0.0)
        return (AxisRef(motor_state=MOTOR_POSITION, ref_val=pos, accel_limit=ac,
                        kp_scale=kp, kv_scale=kd),
                f"POSITION pos={pos} kp={kp} kd={kd} accel={ac}")
    if typ == "pd":   # インピーダンス: ref_val=pos, ref_val_1=vel, ref_val_2=torq(FF)
        pos, vel, torq = n("pos"), n("vel"), n("torq")
        kp, kd = n("kp", 1.0), n("kd", 1.0)
        return (AxisRef(motor_state=MOTOR_POSITION_PD, ref_val=pos, ref_val_1=vel,
                        ref_val_2=torq, kp_scale=kp, kv_scale=kd),
                f"IMPEDANCE pos={pos} vel={vel} torq={torq} kp={kp} kd={kd}")
    if typ == "cpos":  # カスケードPID位置 (FOCTIVE専用): ref_val=pos, accel_limit
        pos, ac = n("pos"), n("accel", 0.0)
        return (AxisRef(motor_state=MOTOR_CASCADE_POS_PID, ref_val=pos, accel_limit=ac,
                        kp_scale=1.0, kv_scale=1.0),
                f"CASCADE_POS pos={pos} accel={ac}")
    if typ == "cvel":  # カスケードPID速度 (FOCTIVE専用): ref_val=vel, accel_limit
        vel, ac = n("vel"), n("accel", 0.0)
        return (AxisRef(motor_state=MOTOR_CASCADE_VEL_PID, ref_val=vel, accel_limit=ac,
                        kp_scale=1.0, kv_scale=1.0),
                f"CASCADE_VEL vel={vel} accel={ac}")
    raise ValueError(f"unknown command type: {typ}")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_html()
        elif self.path == "/status":
            self._json(200, {"last_sent": LAST_SENT["desc"]})
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/cmd":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length) or b"{}")
            rec, desc = build_ref(body.get("type"), body.get("params", {}))
        except (ValueError, TypeError, IndexError, json.JSONDecodeError) as e:
            self._json(400, {"ok": False, "error": str(e)})
            return
        try:
            CMD_Q.put_nowait((rec, desc))
        except queue.Full:
            self._json(503, {"ok": False, "error": "queue full"})
            return
        self._json(200, {"ok": True, "queued": desc})

    def _serve_html(self):
        try:
            with open(HTML_PATH, "rb") as fp:
                body = fp.read()
        except OSError:
            self.send_error(500, "web_controller.html not found")
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
    print(f"web_controller: http://0.0.0.0:{PORT}/")
    httpd.serve_forever()


def axis_ref_bytes(rec):
    return pa.array(list(pack_axis_ref(rec)), type=pa.uint8())


def main():
    threading.Thread(target=serve_http, daemon=True).start()
    node = Node("web_controller")
    print("[web_controller] ready. ブラウザから指令を送ってください。")
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            # tick 毎にキューを drain して送信 (送信は必ずこのメインスレッド)
            while True:
                try:
                    rec, desc = CMD_Q.get_nowait()
                except queue.Empty:
                    break
                node.send_output("motor_commands", axis_ref_bytes(rec))
                LAST_SENT["desc"] = desc
                print(f"[web_controller] sent: {desc}")


if __name__ == "__main__":
    main()
