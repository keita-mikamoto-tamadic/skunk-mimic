#!/usr/bin/env python3
"""robot 側 CUI データビューア (dataflow 非依存 / dora ノードではない)。

`dora topic echo` を subprocess で回して motor_status / imu_data / latency /
state_status の最新値を持ち、端末に 10fps で描き直す。web_monitor と同じ pull 型:

  * yaml に書かない → 開始バリアに影響しない、起動手順に組み込まなくていい
  * 見たいときだけ robot 上で立てて Ctrl-C で閉じられる
  * WiFi を通さない (robot のローカル coordinator に繋ぐ)
  * rich 不使用 (ANSI で画面クリア → 再描画の素朴な CUI)。ノード版
    (data_viewer/data_viewer.py) は受信だけで CPU を食うため、常用はこちら

前提: dataflow に `_unstable_debug.enable_debug_inspection: true`
      (dataflow_mimic.yaml は設定済み)。`dora` CLI が要る。

使い方 (robot 上):
  cd src/python && ROBOT_CONFIG=robot_config/mimic_v2_5.json python3 data_viewer.py
  (uv 不要: stdlib + lib/*_format.py だけ)

環境変数:
  ROBOT_CONFIG    軸名/CAN ID 表示用 (任意、相対パスはリポジトリルート基準)
  DORA_DATAFLOW   固定したい dataflow 名/UUID (既定: 実行中を自動発見)
  DORA_BIN        dora 実行ファイル (既定: PATH か ~/dora/target/release/dora)
"""

import json
import os
import shutil
import subprocess
import sys
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, SCRIPT_DIR)
from lib.axis_data_format import AXIS_ACT_SIZE, unpack_axis_act  # noqa: E402
from lib.sensor_data_format import (  # noqa: E402
    IMU_DATA_SIZE, LATENCY_DATA_SIZE, unpack_imu_data, unpack_latency_data,
)
from lib.enum_def import State  # noqa: E402

RENDER_HZ = 30
TOPICS = {
    "motor_status": "device_control_manager/motor_status",
    "imu_data": "device_control_manager/imu_data",
    "latency": "device_control_manager/latency",
    "state_status": "robot_control_manager/state_status",
}
# moteus Mode (moteus_protocol.h) の主なもの
MOTEUS_MODE = {0: "Stopped", 1: "Fault", 10: "Position", 11: "PosTimeout",
               12: "ZeroVel", 15: "Brake"}


def find_dora():
    cand = os.environ.get("DORA_BIN") or shutil.which("dora") \
        or os.path.expanduser("~/dora/target/release/dora")
    if not (cand and os.path.exists(cand)):
        sys.exit("data_viewer: `dora` CLI が見つかりません。DORA_BIN で指定してください。")
    return cand


DORA = find_dora()


def load_axis_meta():
    """[(name, device_id), ...]。config が無ければ空。"""
    path = os.environ.get("ROBOT_CONFIG", "")
    if not path:
        return "(no config)", []
    if not os.path.isabs(path):
        path = os.path.join(PROJECT_ROOT, path)
    try:
        from lib import robot_config
        cfg = robot_config.load_from_file(path)
        return cfg.robot_name, [(ax.name, ax.device_id) for ax in cfg.axes]
    except Exception as e:  # 表示用なので落とさない
        print(f"data_viewer: config 読み込み失敗 (無視): {e}", file=sys.stderr)
        return "(config error)", []


def discover_dataflow():
    fixed = os.environ.get("DORA_DATAFLOW")
    if fixed:
        return fixed
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


class Latest:
    """topic ごとの最新生バイト (スレッド間共有)。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.raw = {}       # topic key -> bytes
        self.count = {}     # topic key -> 受信数 (レート表示用)
        self.t_last = {}    # topic key -> 最終受信 monotonic

    def put(self, key, raw):
        with self.lock:
            self.raw[key] = raw
            self.count[key] = self.count.get(key, 0) + 1
            self.t_last[key] = time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(self.raw), dict(self.count), dict(self.t_last)


LATEST = Latest()
STOP = threading.Event()


def echo_thread(key, topic):
    """dataflow を発見して `dora topic echo` を回し、落ちたら再発見して追従。"""
    while not STOP.is_set():
        df = discover_dataflow()
        if not df:
            time.sleep(1.0)
            continue
        proc = subprocess.Popen(
            [DORA, "topic", "echo", "-d", df, topic, "--format", "json"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        )
        try:
            for line in proc.stdout:
                if STOP.is_set():
                    break
                try:
                    data = json.loads(line).get("data")
                except json.JSONDecodeError:
                    continue
                if data:
                    LATEST.put(key, bytes(data))
        finally:
            proc.terminate()
        time.sleep(0.5)


def fmt_state(raw):
    if not raw:
        return "----"
    try:
        return State(raw[0]).name
    except ValueError:
        return f"?{raw[0]}"


def render(robot_name, axis_meta, prev_count, prev_t):
    raw, count, t_last = LATEST.snapshot()
    now = time.monotonic()
    lines = []
    A = lines.append

    # ---- header ----
    state = fmt_state(raw.get("state_status"))
    A(f"=== {robot_name}   state: {state}   {time.strftime('%H:%M:%S')} ===")

    # ---- motor ----
    ms = raw.get("motor_status", b"")
    n = len(ms) // AXIS_ACT_SIZE
    A("")
    A(f"{'axis':12s} {'CAN':>4s} {'pos':>9s} {'vel':>9s} {'trq':>8s} {'flt':>3s} {'mode':>11s} {'drops':>6s}")
    for i in range(n):
        a = unpack_axis_act(ms, i * AXIS_ACT_SIZE)
        name, cid = axis_meta[i] if i < len(axis_meta) else (f"#{i}", "-")
        mode = f"{a.mode}:{MOTEUS_MODE.get(a.mode, '?')}"
        warn = " <" if (a.fault or a.silent_ticks or a.mode not in (0, 10)) else ""
        A(f"{name:12s} {cid!s:>4s} {a.position:+9.4f} {a.velocity:+9.4f} {a.torque:+8.3f} "
          f"{a.fault:>3d} {mode:>11s} {a.silent_ticks:>6d}{warn}")
    if n == 0:
        A("  (motor_status なし)")

    # ---- imu ----
    A("")
    im = raw.get("imu_data", b"")
    if len(im) >= IMU_DATA_SIZE:
        d = unpack_imu_data(im)
        A(f"IMU  roll {d.roll:+.4f}  pitch {d.pitch:+.4f}  yaw {d.yaw:+.4f}  [rad]")
        A(f"     gx   {d.gx:+.4f}  gy    {d.gy:+.4f}  gz  {d.gz:+.4f}  [rad/s]")
        A(f"     ax   {d.ax:+.3f}   ay    {d.ay:+.3f}   az  {d.az:+.3f}   [m/s2]")
    else:
        A("IMU  (imu_data なし)")

    # ---- latency ----
    A("")
    lt = raw.get("latency", b"")
    if len(lt) >= LATENCY_DATA_SIZE:
        l = unpack_latency_data(lt)
        A(f"latency  CAN avg {l.can_avg_us:6.0f}us max {l.can_max_us:6.0f}us   "
          f"CTRL avg {l.ctrl_avg_us:6.0f}us max {l.ctrl_max_us:6.0f}us   "
          f"SEND avg {l.send_avg_us:5.0f}us max {l.send_max_us:5.0f}us")
    else:
        A("latency  (なし)")

    # ---- topic rates ----
    A("")
    dt = max(1e-6, now - prev_t)
    parts = []
    for key in TOPICS:
        c = count.get(key, 0)
        hz = (c - prev_count.get(key, 0)) / dt
        age = now - t_last.get(key, 0) if key in t_last else float("inf")
        stale = "" if age < 0.5 else " (stale)"
        parts.append(f"{key} {hz:5.0f}Hz{stale}")
    A("rate: " + "  ".join(parts))
    A("Ctrl-C で終了")

    # 画面クリアして描く (ANSI: home + clear-to-end)
    sys.stdout.write("\x1b[H\x1b[J" + "\n".join(lines) + "\n")
    sys.stdout.flush()
    return count, now


def main():
    robot_name, axis_meta = load_axis_meta()
    for key, topic in TOPICS.items():
        threading.Thread(target=echo_thread, args=(key, topic), daemon=True).start()
    prev_count, prev_t = {}, time.monotonic()
    try:
        while True:
            prev_count, prev_t = render(robot_name, axis_meta, prev_count, prev_t)
            time.sleep(1.0 / RENDER_HZ)
    except KeyboardInterrupt:
        STOP.set()
        print()


if __name__ == "__main__":
    main()
