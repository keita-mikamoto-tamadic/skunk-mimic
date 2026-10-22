"""FOCTIVE 操作 GUI ノード (foctive_web_gui)。

旧 foctive_controller(端末) + web_controller(指令) + web_monitor(表示) を 1 ノードに
まとめたブラウザ GUI。dora が run_node.bash 経由で直接 spawn する (dynamic 不要)。

  ブラウザ ⇄ HTTP/SSE ⇄ foctive_web_gui ──motor_commands / settings_request──▶ DCM ⇄ CAN
                              ▲──── motor_status / settings_result / param_dump ────┘

通信は dora トピックのみ。DCM 側は何も知らないので、ロボットを FOCTIVE で組んだ
dataflow にもそのまま差し込める:
  - 単体テスト: motor_commands を DCM に配線 (操作タブ有効)
  - ロボット  : motor_commands は RCM が持つので FOCTIVE_WEB_GUI_CONTROL=0 にして
                settings_request だけ配線 (パラメータタブ + モニタのみ)。
                state_status を配線すれば RCM が OFF のときだけ設定を許可する。

設定モード (msg bit 15) はファーム側でサーボ OFF として扱われ、DCM も処理中は
ループをブロックする (calib は最大 10s)。そのため設定操作は読み出しも含め
「全軸 OFF のときだけ」許可し、同時に 1 件ずつ直列に処理する。
DCM は成功時 param_dump → settings_result の順で返す。

送信 (send_output) は必ず node メインスレッドから。HTTP スレッドはキューに積み、
設定要求は threading.Event で結果を待つ。

環境変数:
  ROBOT_CONFIG              robot_config JSON (相対パスはリポジトリルート基準)
  FOCTIVE_WEB_GUI_PORT      HTTP ポート (既定 8780)
  FOCTIVE_WEB_GUI_CONTROL   1=操作タブ有効 (既定) / 0=無効 (motor_commands を送らない)
  FOCTIVE_WEB_GUI_ENABLE    1=開始/OFF ボタンで "enable" (uint8 1/0) を送る。
                            motor_commands を別ノード (position_exchanger 等) が持つ構成用。
                            CONTROL=0 と組み合わせる。
"""

import json
import os
import queue
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pyarrow as pa
from dora import Node

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lib.axis_data_format import (  # noqa: E402  自動生成(axis_data.json 正本)
    AXIS_ACT_SIZE, AxisRef, pack_axis_ref, unpack_axis_act,
    SettingsRequest, pack_settings_request, unpack_settings_result,
    unpack_param_scalars, PARAM_SCALARS_FIELDS, PARAM_SCALARS_WIRE_INDEX,
)
from lib import robot_config  # noqa: E402

PORT = int(os.environ.get("FOCTIVE_WEB_GUI_PORT", "8780"))
CONTROL_ENABLED = os.environ.get("FOCTIVE_WEB_GUI_CONTROL", "1") != "0"
ENABLE_OUTPUT = os.environ.get("FOCTIVE_WEB_GUI_ENABLE", "0") == "1"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
UPLOT_DIR = os.path.join(PROJECT_ROOT, "tools", "gui")  # web_monitor 同梱の uPlot を共用

PUB_INTERVAL_S = 0.033  # SSE 配信 ~30Hz (motor_status は 1kHz で来ても間引く)
# FOCTIVE は制御モード同士の直接遷移を無視する (必ず Idle 経由)。モードが変わる軸は
# 先に OFF を送り、DCM が確実に 1 回以上送信するだけ待ってから新指令を送る。
# DCM は tick 毎に最新指令だけを送るので、間隔が tick 以下だと OFF が潰れる。
IDLE_HOLD_S = 0.05

# MotorState (enum_def.hpp と一致)
MOTOR_OFF = 0
MOTOR_POSITION = 2
MOTOR_VELOCITY = 3
MOTOR_CURRENT = 6
MOTOR_VOLTAGE = 7
MOTOR_POSITION_PD = 8
MOTOR_CASCADE_POS_PID = 9
MOTOR_CASCADE_VEL_PID = 10

STATE_OFF = 0  # robot State::OFF (state_status 配線時のみ使用)

# 設定 op → (SettingsCmd, DCM 応答待ち timeout[s])。timeout は DCM 側 timeout + 余裕
SETTINGS_OPS = {
    "pread":   (104, 1.0),
    "pwrite":  (103, 1.0),
    "readall": (102, 2.0),
    "save":    (100, 2.0),
    "load":    (101, 2.0),
    "calib":   (1, 13.0),   # ファーム timeout 10s + 余裕
    "setpos":  (110, 1.0),
}

# uint32 として扱う wire param_index (それ以外は float の生ビット)
INT_PARAM_INDICES = {0, 1, 2, 3, 4, 5, 6}
LUT_INDEX = PARAM_SCALARS_WIRE_INDEX["elec_angle_ofs"]  # 書き込み不可


def _load_config():
    path = os.environ.get("ROBOT_CONFIG", "robot_config/foctive_motor_test.json")
    if not os.path.isabs(path):
        path = os.path.join(PROJECT_ROOT, path)
    return robot_config.load_from_file(path)


CFG = _load_config()
AXES = CFG.axes
AXIS_COUNT = len(AXES)


def f2u(v):
    return struct.unpack("<I", struct.pack("<f", float(v)))[0]


def u2f(u):
    return struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0]


def decode_param(index, raw):
    return raw if index in INT_PARAM_INDICES else u2f(raw)


def encode_param(index, value):
    return int(float(value)) & 0xFFFFFFFF if index in INT_PARAM_INDICES else f2u(value)


def build_ref(typ, p):
    """指令 type + params → (AxisRef, desc)。マッピングは web_controller と一致。"""
    p = p or {}

    def n(key, default=0.0):
        v = p.get(key)
        return float(v) if isinstance(v, (int, float)) else float(default)

    if typ == "off":
        return AxisRef(motor_state=MOTOR_OFF), "OFF"
    if typ == "v":
        d, q, fr = n("volt_d"), n("volt_q"), n("vir_ang_freq")
        return (AxisRef(motor_state=MOTOR_VOLTAGE, ref_val=d, ref_val_1=q, ref_val_2=fr,
                        kp_scale=1.0, kv_scale=1.0),
                f"VOLTAGE d={d} q={q} freq={fr}")
    if typ == "c":
        d, q = n("cur_d"), n("cur_q")
        return (AxisRef(motor_state=MOTOR_CURRENT, ref_val=d, ref_val_1=q,
                        kp_scale=1.0, kv_scale=1.0),
                f"CURRENT d={d} q={q}")
    if typ == "vel":
        vel, kp, kd, ac = n("vel"), n("kp", 1.0), n("kd", 1.0), n("accel")
        return (AxisRef(motor_state=MOTOR_VELOCITY, ref_val=vel, accel_limit=ac,
                        kp_scale=kp, kv_scale=kd),
                f"VELOCITY vel={vel} kp={kp} kd={kd} accel={ac}")
    if typ == "p":
        pos, kp, kd, ac = n("pos"), n("kp", 1.0), n("kd", 1.0), n("accel")
        return (AxisRef(motor_state=MOTOR_POSITION, ref_val=pos, accel_limit=ac,
                        kp_scale=kp, kv_scale=kd),
                f"POSITION pos={pos} kp={kp} kd={kd} accel={ac}")
    if typ == "pd":
        pos, vel, torq = n("pos"), n("vel"), n("torq")
        kp, kd = n("kp", 1.0), n("kd", 1.0)
        return (AxisRef(motor_state=MOTOR_POSITION_PD, ref_val=pos, ref_val_1=vel,
                        ref_val_2=torq, kp_scale=kp, kv_scale=kd),
                f"IMPEDANCE pos={pos} vel={vel} torq={torq} kp={kp} kd={kd}")
    if typ == "cpos":
        pos, ac = n("pos"), n("accel")
        return (AxisRef(motor_state=MOTOR_CASCADE_POS_PID, ref_val=pos, accel_limit=ac,
                        kp_scale=1.0, kv_scale=1.0),
                f"CASCADE_POS pos={pos} accel={ac}")
    if typ == "cvel":
        vel, ac = n("vel"), n("accel")
        return (AxisRef(motor_state=MOTOR_CASCADE_VEL_PID, ref_val=vel, accel_limit=ac,
                        kp_scale=1.0, kv_scale=1.0),
                f"CASCADE_VEL vel={vel} accel={ac}")
    raise ValueError(f"unknown command type: {typ}")


def parse_axis(body, allow_all):
    axis = body.get("axis", "all" if allow_all else 0)
    if allow_all and axis == "all":
        return "all"
    if isinstance(axis, int) and 0 <= axis < AXIS_COUNT:
        return axis
    raise ValueError(f"axis must be 0..{AXIS_COUNT - 1}: {axis!r}")


# ---- HTTP スレッド ⇄ node メインスレッド ------------------------------------

CMD_Q = queue.Queue(maxsize=64)   # (axis, AxisRef, desc)
SET_Q = queue.Queue(maxsize=8)    # SettingsJob


class SettingsJob:
    def __init__(self, op, axis, index=0, value=0.0):
        self.op = op
        self.axis = axis
        self.cmd, self.timeout = SETTINGS_OPS[op]
        self.index = index
        self.value = value
        self.done = threading.Event()
        self.result = None   # dict (HTTP 応答)
        self.dump = None     # param_dump (readall/load)
        self.deadline = 0.0

    def finish(self, result):
        self.result = result
        self.done.set()


class Shared:
    """SSE 配信用の共有状態 (node スレッドが書き、HTTP スレッドが読む)。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.servo = ["OFF"] * AXIS_COUNT   # 各軸に最後に送った指令
        self.robot_state = None             # state_status (未配線なら None)
        self.busy = None                    # 処理中の設定 op
        self.last = "(none)"
        self.enabled = False                # FOCTIVE_WEB_GUI_ENABLE 時の開始状態


SHARED = Shared()
ENABLE_Q = queue.Queue(maxsize=16)   # bool (開始/停止)。送信は node スレッド


class Broadcaster:
    """SSE 購読者へ最新スナップショットを配る (web_monitor と同じ)。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._subs = set()
        self._latest = None

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

    def publish(self, payload):
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

STATIC = {
    "/": (SCRIPT_DIR, "foctive_web_gui.html", "text/html; charset=utf-8"),
    "/index.html": (SCRIPT_DIR, "foctive_web_gui.html", "text/html; charset=utf-8"),
    "/uPlot.iife.min.js": (UPLOT_DIR, "uPlot.iife.min.js", "application/javascript"),
    "/uPlot.min.css": (UPLOT_DIR, "uPlot.min.css", "text/css"),
}


def config_json():
    params = []
    for name in PARAM_SCALARS_FIELDS:
        idx = PARAM_SCALARS_WIRE_INDEX[name]
        params.append({"name": name, "index": idx,
                       "type": "lut" if idx == LUT_INDEX
                       else ("int" if idx in INT_PARAM_INDICES else "float")})
    params.sort(key=lambda p: p["index"])
    return {
        "robot_name": CFG.robot_name,
        "protocol": CFG.protocol,
        "control": CONTROL_ENABLED,
        "enable": ENABLE_OUTPUT,
        "axes": [{"name": ax.name, "device_id": ax.device_id} for ax in AXES],
        "params": params,
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self._serve_stream()
        elif self.path == "/config":
            self._json(200, config_json())
        elif self.path in STATIC:
            self._serve_static(*STATIC[self.path])
        else:
            self.send_error(404)

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError) as e:
            self._json(400, {"ok": False, "error": str(e)})
            return
        if self.path == "/cmd":
            self._post_cmd(body)
        elif self.path == "/settings":
            self._post_settings(body)
        elif self.path == "/enable":
            self._post_enable(body)
        else:
            self.send_error(404)

    def _post_enable(self, body):
        if not ENABLE_OUTPUT:
            self._json(403, {"ok": False, "error": "enable 出力は無効 (FOCTIVE_WEB_GUI_ENABLE=0)"})
            return
        try:
            ENABLE_Q.put_nowait(bool(body.get("on")))
        except queue.Full:
            self._json(503, {"ok": False, "error": "queue full"})
            return
        self._json(200, {"ok": True})

    def _post_cmd(self, body):
        if not CONTROL_ENABLED:
            self._json(403, {"ok": False, "error": "操作は無効 (FOCTIVE_WEB_GUI_CONTROL=0)"})
            return
        try:
            axis = parse_axis(body, allow_all=True)
            rec, desc = build_ref(body.get("type"), body.get("params", {}))
        except (ValueError, TypeError) as e:
            self._json(400, {"ok": False, "error": str(e)})
            return
        try:
            CMD_Q.put_nowait((axis, rec, desc))
        except queue.Full:
            self._json(503, {"ok": False, "error": "queue full"})
            return
        self._json(200, {"ok": True, "queued": desc})

    def _post_settings(self, body):
        try:
            op = body.get("op")
            if op not in SETTINGS_OPS:
                raise ValueError(f"unknown op: {op!r}")
            axis = parse_axis(body, allow_all=False)
            index = int(body.get("index", 0))
            if op == "pwrite" and index == LUT_INDEX:
                raise ValueError("LUT (index 8) は書き込み不可")
            value = float(body.get("value", 0.0))
            job = SettingsJob(op, axis, index, value)
        except (ValueError, TypeError) as e:
            self._json(400, {"ok": False, "error": str(e)})
            return
        try:
            SET_Q.put_nowait(job)
        except queue.Full:
            self._json(503, {"ok": False, "error": "設定キューが満杯"})
            return
        # node 側が deadline で必ず finish するが、node 停止時に備えて上限を置く
        if not job.done.wait(job.timeout + 5.0):
            self._json(504, {"ok": False, "error": "node 無応答"})
            return
        self._json(200, job.result)

    def _serve_static(self, directory, fname, ctype):
        try:
            with open(os.path.join(directory, fname), "rb") as f:
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
    print(f"[foctive_web_gui] http://0.0.0.0:{PORT}/")
    httpd.serve_forever()


# ---- node メインスレッド ------------------------------------------------------

def u8_array(b):
    return pa.array(list(b), type=pa.uint8())


class GuiNode:
    def __init__(self):
        self.node = Node()
        self.refs = [AxisRef(motor_state=MOTOR_OFF) for _ in range(AXIS_COUNT)]
        self.pending = None   # 処理中の SettingsJob
        self.deferred = []    # Idle 経由待ちの指令 [(due, axis, AxisRef, desc)]
        self.axes_json = []
        self.last_pub = 0.0

    # 設定を許可してよいか。理由文字列 (None なら OK)
    def settings_block_reason(self):
        if CONTROL_ENABLED and (self.deferred or
                                any(r.motor_state != MOTOR_OFF for r in self.refs)):
            return "サーボ中は設定不可 (先に OFF)"
        with SHARED.lock:
            st = SHARED.robot_state
            if ENABLE_OUTPUT and SHARED.enabled:
                return "外部制御が動作中は設定不可 (先に OFF)"
        if st is not None and st != STATE_OFF:
            return f"ロボット状態が OFF ではない (state={st})"
        return None

    def apply(self, targets, rec, desc):
        for i in targets:
            self.refs[i] = rec
        with SHARED.lock:
            for i in targets:
                SHARED.servo[i] = desc

    def drain_commands(self):
        dirty = False
        now = time.monotonic()
        # Idle 経由待ちが明けた指令を適用
        due = [d for d in self.deferred if d[0] <= now]
        self.deferred = [d for d in self.deferred if d[0] > now]
        for _, i, rec, desc in due:
            self.apply([i], rec, desc)
            dirty = True
        while True:
            try:
                axis, rec, desc = CMD_Q.get_nowait()
            except queue.Empty:
                break
            targets = list(range(AXIS_COUNT)) if axis == "all" else [axis]
            # 新しい指令で、その軸の未適用の指令は上書き (OFF は即時優先)
            self.deferred = [d for d in self.deferred if d[1] not in targets]
            direct, via_idle = [], []
            for i in targets:
                cur = self.refs[i].motor_state
                if (rec.motor_state != MOTOR_OFF and cur != MOTOR_OFF
                        and cur != rec.motor_state):
                    via_idle.append(i)
                else:
                    direct.append(i)
            self.apply(direct, rec, desc)
            if via_idle:
                self.apply(via_idle, AxisRef(motor_state=MOTOR_OFF), "OFF (モード切替中)")
                self.deferred += [(now + IDLE_HOLD_S, i, rec, desc) for i in via_idle]
            label = "ALL" if axis == "all" else f"[{axis}] {AXES[axis].name}"
            with SHARED.lock:
                SHARED.last = f"{label}: {desc}"
            print(f"[foctive_web_gui] sent: {label}: {desc}"
                  + (f" (Idle 経由: {via_idle})" if via_idle else ""))
            dirty = True
        if dirty:
            self.node.send_output(
                "motor_commands",
                u8_array(b"".join(pack_axis_ref(r) for r in self.refs)))

    def drain_enable(self):
        while True:
            try:
                on = ENABLE_Q.get_nowait()
            except queue.Empty:
                return
            self.node.send_output("enable", u8_array(bytes([1 if on else 0])))
            with SHARED.lock:
                SHARED.enabled = on
                SHARED.last = "開始" if on else "OFF (停止)"
            print(f"[foctive_web_gui] enable={int(on)}")

    def dispatch_settings(self):
        if self.pending is not None:
            if time.monotonic() > self.pending.deadline:
                self.finish_pending({"ok": False, "error": "timeout (DCM 応答なし)"})
            return
        try:
            job = SET_Q.get_nowait()
        except queue.Empty:
            return
        reason = self.settings_block_reason()
        if reason:
            job.finish({"ok": False, "error": reason})
            return
        if job.op == "pwrite":
            raw = encode_param(job.index, job.value)
        elif job.op in ("calib", "setpos"):
            raw = f2u(job.value)
        else:
            raw = 0
        req = SettingsRequest(device_id=AXES[job.axis].device_id, cmd=job.cmd,
                              param_index=job.index, value=raw)
        job.deadline = time.monotonic() + job.timeout
        self.pending = job
        with SHARED.lock:
            SHARED.busy = job.op
        self.node.send_output("settings_request", u8_array(pack_settings_request(req)))
        print(f"[foctive_web_gui] settings: {job.op} axis={job.axis} "
              f"index={job.index} value={job.value}")

    def finish_pending(self, result):
        job = self.pending
        self.pending = None
        with SHARED.lock:
            SHARED.busy = None
        job.finish(result)

    def on_settings_result(self, raw):
        job = self.pending
        if job is None:
            return
        res = unpack_settings_result(raw)
        if res.cmd != job.cmd:
            return
        if not res.ok:
            self.finish_pending({"ok": False, "error": f"{job.op} 失敗 (CAN timeout/エラー)"})
            return
        out = {"ok": True, "op": job.op}
        if job.op in ("pread", "pwrite"):
            out["index"] = res.param_index
            out["value"] = decode_param(res.param_index, res.value)
            if job.op == "pwrite":
                out["old_value"] = decode_param(res.param_index, res.old_value)
        elif job.op == "calib":
            out["pos"] = u2f(res.value)
        elif job.op == "setpos":
            out["offset"] = u2f(res.value)
        elif job.op in ("readall", "load"):
            if job.dump is None:
                self.finish_pending({"ok": False, "error": "param_dump 未受信"})
                return
            out["params"] = job.dump
        self.finish_pending(out)

    def on_param_dump(self, raw):
        if self.pending is None:
            return
        ps = unpack_param_scalars(raw)
        dump = {}
        for name in PARAM_SCALARS_FIELDS:
            idx = PARAM_SCALARS_WIRE_INDEX[name]
            v = getattr(ps, name)
            dump[name] = list(v) if idx == LUT_INDEX else decode_param(idx, v)
        self.pending.dump = dump

    def on_motor_status(self, raw):
        axes = []
        for i in range(len(raw) // AXIS_ACT_SIZE):
            a = unpack_axis_act(raw, i * AXIS_ACT_SIZE)
            axes.append({"position": a.position, "velocity": a.velocity,
                         "torque": a.torque, "cur_d": a.cur_d, "cur_q": a.cur_q,
                         "fault": a.fault})
        self.axes_json = axes

    def publish(self):
        now = time.monotonic()
        if now - self.last_pub < PUB_INTERVAL_S:
            return
        self.last_pub = now
        with SHARED.lock:
            snap = {"t": time.time(), "axes": self.axes_json,
                    "servo": list(SHARED.servo), "state": SHARED.robot_state,
                    "busy": SHARED.busy, "last": SHARED.last,
                    "enabled": SHARED.enabled if ENABLE_OUTPUT else None}
        BROADCAST.publish(json.dumps(snap))

    def run(self):
        print(f"[foctive_web_gui] ready: {CFG.robot_name} ({AXIS_COUNT} axes, "
              f"control={'on' if CONTROL_ENABLED else 'off'})")
        while True:
            event = self.node.next(timeout=0.02)
            if event is not None:
                if event["type"] == "STOP":
                    break
                if event["type"] == "INPUT":
                    eid = event["id"]
                    raw = bytes(event["value"].to_numpy(zero_copy_only=False))
                    if eid == "motor_status":
                        self.on_motor_status(raw)
                    elif eid == "settings_result":
                        self.on_settings_result(raw)
                    elif eid == "param_dump":
                        self.on_param_dump(raw)
                    elif eid == "state_status" and raw:
                        with SHARED.lock:
                            SHARED.robot_state = raw[0]
            if CONTROL_ENABLED:
                self.drain_commands()
            if ENABLE_OUTPUT:
                self.drain_enable()
            self.dispatch_settings()
            self.publish()
        if self.pending is not None:
            self.finish_pending({"ok": False, "error": "node 停止"})


def main():
    threading.Thread(target=serve_http, daemon=True).start()
    GuiNode().run()


if __name__ == "__main__":
    main()
