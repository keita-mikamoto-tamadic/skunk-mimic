#!/usr/bin/env python3
"""Motor monitor ブリッジ: dora topic echo --format json → SSE でブラウザGUIへ。

ブラウザは dora の WS topic frame(bincode シリアライズされた InterDaemonEvent)を
解けない。このブリッジが `dora topic echo --format json`(dora 側で正しくデコード済み)を
subprocess で受け、AxisAct を素直な JSON にして SSE(Server-Sent Events)でブラウザへ流す。
web_monitor.html も同オリジンで配信するので CORS / file:// の問題も無い。標準ライブラリのみ。

  # ロボット側(coordinator/daemon/dataflow が動いている前提)で:
  python tools/motor_bridge.py
  # ブラウザで http://<robot-ip>:8765/ を開く

前提: dataflow に _unstable_debug.enable_debug_inspection: true (topic 購読に必須)。
"""
import argparse
import json
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 生成された AxisAct フォーマット(単一正本)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "python"))
from lib.data_format import unpack_axis_act, AXIS_ACT_SIZE  # noqa: E402

HTML_PATH = os.path.join(os.path.dirname(__file__), "web_monitor.html")
CFG = {}  # main で設定


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_html()
        elif self.path.startswith("/stream"):
            self._serve_stream()
        else:
            self.send_error(404)

    def _serve_html(self):
        try:
            with open(HTML_PATH, "rb") as f:
                body = f.read()
        except OSError as e:
            self.send_error(500, str(e))
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")  # file:// からでも可
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        cmd = [CFG["dora"], "topic", "echo", "-d", CFG["dataflow"],
               CFG["topic"], "--format", "json"]
        env = {**os.environ, "DORA_COORDINATOR_ADDR": CFG["coordinator"]}
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, env=env, text=True)
        try:
            for line in proc.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    ev = json.loads(line)
                except json.JSONDecodeError:
                    continue
                data = ev.get("data")
                if not isinstance(data, list):
                    continue
                raw = bytes(int(b) & 0xFF for b in data)
                n = len(raw) // AXIS_ACT_SIZE
                axes = []
                for i in range(n):
                    a = unpack_axis_act(raw, i * AXIS_ACT_SIZE)
                    axes.append({
                        "position": a.position, "velocity": a.velocity,
                        "torque": a.torque, "cur_d": a.cur_d, "cur_q": a.cur_q,
                        "fault": a.fault,
                    })
                payload = json.dumps({"axes": axes})
                self.wfile.write(f"data: {payload}\n\n".encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass  # ブラウザが閉じた
        finally:
            proc.terminate()

    def log_message(self, *args):
        pass  # アクセスログを黙らせる


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataflow", default="foctive")
    ap.add_argument("--topic", default="device_control_manager/motor_status")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--coordinator", default="127.0.0.1",
                    help="coordinator IP (ブリッジから見たアドレス)")
    ap.add_argument("--dora", default=os.path.expanduser("~/dora/target/release/dora"),
                    help="dora バイナリのパス(alias は subprocess で効かないため)")
    args = ap.parse_args()
    CFG.update(vars(args))

    print(f"bridge listening: http://0.0.0.0:{args.port}/")
    print(f"  dataflow={args.dataflow}  topic={args.topic}  coordinator={args.coordinator}")
    print(f"  ブラウザで http://<this-host-ip>:{args.port}/ を開く")
    try:
        ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
