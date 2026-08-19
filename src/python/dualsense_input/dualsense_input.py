"""DualSense (PS5) で state_command を送る dora ノード。dummy_input のパッド版。

robot_web_gui と同じく RCM の状態機械を経由する (motor_commands を直接送る
web_controller とは別物)。入力トピックは持たない (出力のみ)。

state_command (ボタン、イベント駆動) に加えて drive_command (左スティック → 前後/旋回、
右スティック上下 → 姿勢 (重心高さ) の変化レート) を約 50Hz で送り続ける。drive_command は
stabilizer の外側ループの目標ホイール速度と posture IK の目標高さに入る
(詳細は command_data.json の doc)。

## 設計上の約束

**BT 接続をノードの起動条件にしない。** パッドが繋がっていなくても起動し、
実行中に切れても落ちない。dynamic ノードは全 attach まで dataflow を開始バリアで
待たせるため、ここで落ちると dataflow ごと止まる。

**切断時、state_command は何も送らない** (RCM の状態を維持。勝手に SERVO_OFF /
STOP を送らない)。**走行指令 (drive_command) は 0 に戻して送り続ける** — 倒しっぱなしの
値が残ると暴走するため。プロセスごと死んだ場合はこれも送れないので、受け側
(stabilizer) にも 300ms 途絶で 0 扱いの網がある。

**送信はメインスレッドのみ。** パッドの読み取りは別スレッドでキューに積み、
dora のイベントループ側で drain して送る (robot_web_gui と同じ方式)。

## 割り当て (物理ボタン名。js の番号はドライバ依存なので書かない)

  ○                     SERVO_ON              OFF のみ
  △                     READY                 STOP のみ
  □                     RUN                   READY + 補間完了のみ
  ×                     STOP                  OFF 以外
  L1+L3                 SERVO_OFF             常時 (緊急停止)
  Create+Options        INIT_POSITION_RESET   OFF のみ (ストッパー押し当て後に!)
  左スティック上下       drive_command.forward 上=前進。正規化 -1..1、デッドゾーン 0.05
  左スティック左右       drive_command.yaw     右=右旋回。正規化 -1..1、デッドゾーン 0.05
                        (何 rad/s になるかは制御側 kMaxDriveWheelVel / kMaxYawWheelDiff)
  右スティック上下       drive_command.height  上=重心を上げる。正規化レート -1..1、デッドゾーン 0.05
                        (何 m/s になるかは制御側 kMaxComHeightVel。倒立点を変えずに hip/knee が動く)

右の条件は RCM 側のガード (robot_control_manager.cpp の HandleStateCommand)。
ここでは弾かずに送るだけで、通らなければ RCM が黙って無視する。

同時押しは「両方が押されている状態で、後から押した方の押下イベント」で 1 回だけ
発火する。同時押しに使うボタンは単押しに割り当てていないので誤爆しない。

**ボタン番号はドライバで変わる** (ロボットの hid-generic: □0 ×1 ○2 △3 L1 4 L3 10、
PC の hid_playstation: ×0 ○1 △2 □3 L1 4 L3 11)。番号をハードコードせず、接続のたびに
lib/dualsense_map が sysfs のドライバ名 + JSIOCGBTNMAP から {物理名 → 番号} を解決する
(接続ログに出る)。実測根拠は scripts/dualsense_monitor/README.md (hid-generic の番号)。

## 使い方

  cd src/python && uv run dualsense_input/dualsense_input.py

  DUALSENSE_DEV      デバイス (既定 /dev/input/js0)
  DUALSENSE_SCHEME   generic | playstation — ドライバ自動判定を上書き (通常不要)
"""

import os
import queue
import select
import struct
import sys
import threading
import time

import pyarrow as pa
from dora import Node

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from lib.enum_def import StateCommand  # noqa: E402
from lib.command_data_format import DriveCommand, pack_drive_command  # noqa: E402
from lib import dualsense_map as dsm  # noqa: E402

# struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
JS_FMT = "<IhBB"
JS_SIZE = 8
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

DEV = os.environ.get("DUALSENSE_DEV", "/dev/input/js0")
REOPEN_INTERVAL = 1.0     # 切断中の再オープン試行間隔 [s]
LOOP_TIMEOUT = 0.02       # dora イベント待ち [s] = drain 周期 = drive_command 送信周期 (50Hz)

AXIS_MAX = 32767
STICK_DEADZONE = 0.05     # 正規化後のデッドゾーン

# 割り当ては js のボタン番号ではなく物理ボタン名で書く。番号は掴んでいる HID ドライバで
# 変わる (ロボットの hid-generic: □0 ×1 ○2 △3 L1 4 L3 10 / PC の hid_playstation:
# ×0 ○1 △2 □3 L1 4 L3 11) ので、接続のたびに lib/dualsense_map が JSIOCGBTNMAP と
# sysfs のドライバ名から {物理名 → 番号} を解決する。左スティックの軸番号も同様に引く。
#
# 単押し: {物理名: (StateCommand, 表示名)}
SINGLE = {
    dsm.CIRCLE:   (StateCommand.SERVO_ON, "SERVO_ON"),
    dsm.TRIANGLE: (StateCommand.READY,    "READY"),
    dsm.SQUARE:   (StateCommand.RUN,      "RUN"),
    dsm.CROSS:    (StateCommand.STOP,     "STOP"),
}
# 同時押し: {(物理名, 物理名): (StateCommand, 表示名)}
COMBO = {
    (dsm.L1, dsm.L3):          (StateCommand.SERVO_OFF, "SERVO_OFF"),
    (dsm.CREATE, dsm.OPTIONS): (StateCommand.INIT_POSITION_RESET, "INIT_POSITION_RESET"),
}
COMBO_MEMBERS = {b for pair in COMBO for b in pair}
USED_BUTTONS = set(SINGLE) | COMBO_MEMBERS


def resolve_map(f, dev):
    """開いた js デバイスから物理名→番号の対応を作る。ioctl が失敗したらロボット既定
    (hid-generic 番号) に倒して警告する (起動は止めない)。"""
    try:
        m = dsm.DualSenseMap.from_fd(f.fileno(), dev)
    except OSError as e:
        m = dsm.DualSenseMap.default_generic()
        log(f"WARNING: button map ioctl failed ({e}); assuming hid-generic numbering")
    missing = sorted(USED_BUTTONS - set(m.index_of))
    if missing:
        log(f"WARNING: buttons not found on this device: {missing} — those commands won't fire")
    return m


def stick_to_norm(raw, invert=False):
    """js の生値 (-32767..32767) → 正規化操作量 -1..1。

    物理量へのスケールはここでは持たない (制御側 angle_pid の
    kMaxDriveWheelVel / kMaxYawWheelDiff が正)。デッドゾーンは「切る」の
    ではなく閾値分を引いて 0..1 に張り直す (閾値をまたいだ瞬間の段差をなくす)。
    """
    x = (-raw if invert else raw) / AXIS_MAX
    x = max(-1.0, min(1.0, x))
    if abs(x) < STICK_DEADZONE:
        return 0.0
    mag = (abs(x) - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)
    return mag if x > 0 else -mag

def log(msg):
    print(f"[dualsense_input] {msg}", flush=True)


def reader_thread(dev, out_q, drive, stop_ev):
    """js0 を読んで、ボタン発火 (StateCommand, 名前) をキューに積み、
    左スティックの値を drive["forward"] に反映する。

    デバイスが無くても、途中で消えても、決して例外で抜けない。
    """
    f = None
    m = None                  # 接続中のボタン/軸マップ (lib/dualsense_map.DualSenseMap)
    pressed = set()           # 押下中の物理名
    next_try = 0.0

    def close(reason):
        nonlocal f, m
        if f is not None:
            try:
                f.close()
            except OSError:
                pass
        f = None
        m = None
        pressed.clear()
        # 切断時: state_command は送らない (RCM の状態を維持) が、
        # 走行指令は 0 に戻す (倒しっぱなしの値を残さない)
        drive["forward"] = 0.0
        drive["yaw"] = 0.0
        drive["height"] = 0.0
        log(f"disconnected ({reason}) — 走行指令 0、状態は維持。再接続を待つ")

    while not stop_ev.is_set():
        if f is None:
            now = time.monotonic()
            if now < next_try:
                stop_ev.wait(min(REOPEN_INTERVAL, next_try - now))
                continue
            next_try = now + REOPEN_INTERVAL
            try:
                f = open(dev, "rb")
            except OSError:
                continue
            pressed.clear()
            m = resolve_map(f, dev)
            log(f"connected: {dev} — {m.describe()}")

        try:
            if not select.select([f], [], [], 0.1)[0]:
                continue
            data = f.read(JS_SIZE)
        except OSError as e:
            close(str(e))
            continue
        if not data or len(data) < JS_SIZE:
            close("device removed")
            continue

        _, value, etype, number = struct.unpack(JS_FMT, data)
        if etype & JS_EVENT_AXIS and number == m.axis_ly:
            # 軸は INIT (オープン直後の現在値) も受ける — 実際の位置なので
            drive["forward"] = stick_to_norm(value, invert=True)  # 上=前進=正 (上が負の生値)
            continue
        if etype & JS_EVENT_AXIS and number == m.axis_lx:
            drive["yaw"] = stick_to_norm(value)                   # 右=右旋回=正
            continue
        if etype & JS_EVENT_AXIS and number == m.axis_index_of.get(dsm.AX_RY, -1):
            drive["height"] = stick_to_norm(value, invert=True)   # 上=上げる=正
            continue
        if etype & JS_EVENT_INIT or not (etype & JS_EVENT_BUTTON):
            # ボタンの INIT は無視 (押下イベントでのみ発火させる)。他の軸も未使用
            continue
        name = m.name_of.get(number)
        if name is None:
            continue                      # この割り当てで使わないボタン (タッチパッド等)
        if not value:
            pressed.discard(name)
            continue

        pressed.add(name)
        fired = False
        # 同時押しを先に判定。今押したボタンが組の一方で、もう一方も押下中なら発火
        for (a, b), entry in COMBO.items():
            other = b if name == a else (a if name == b else None)
            if other is not None and other in pressed:
                out_q.put(entry)
                fired = True
        if fired:
            continue
        if name in SINGLE and name not in COMBO_MEMBERS:
            out_q.put(SINGLE[name])


def main():
    node = Node("dualsense_input")
    out_q = queue.Queue()
    drive = {"forward": 0.0, "yaw": 0.0, "height": 0.0}   # reader が書き、main が送る (GIL で atomic)
    stop_ev = threading.Event()
    threading.Thread(target=reader_thread, args=(DEV, out_q, drive, stop_ev),
                     daemon=True).start()

    log(f"device={DEV} (未接続でも起動する / 切断しても落ちない)")
    for b, (_, name) in SINGLE.items():
        log(f"  {dsm.glyph(b):<15} -> {name}")
    for (a, b), (_, name) in COMBO.items():
        log(f"  {dsm.glyph(a)}+{dsm.glyph(b):<12} -> {name}")
    log("  (js ボタン番号はドライバ依存。接続時に実機の対応を解決して表示する)")
    log(f"  左スティック上下  -> drive_command.forward (上=前進, 正規化 -1..1)")
    log(f"  左スティック左右  -> drive_command.yaw     (右=右旋回, 正規化 -1..1)")
    log(f"  右スティック上下  -> drive_command.height  (上=重心を上げる, 正規化レート -1..1)")
    log(f"  (deadzone {STICK_DEADZONE}; 物理スケールは制御側 angle_pid が持つ)")

    # `for event in node` (無期限 recv) ではなく timeout 付きで回す:
    # daemon が先に落ちる等でイベントが二度と来なくなっても LOOP_TIMEOUT ごとに
    # Python に制御が戻り、Ctrl-C が効く & キューを drain できる。
    try:
        while True:
            event = node.next(timeout=LOOP_TIMEOUT)
            if event is not None and event["type"] == "STOP":
                log("STOP received")
                break
            while True:
                try:
                    cmd, name = out_q.get_nowait()
                except queue.Empty:
                    break
                node.send_output("state_command",
                                 pa.array([int(cmd)], type=pa.uint8()))
                log(f"sent: {name}")
            # 走行指令は毎周期 (~50Hz) 送る。受け側は 300ms 途絶で 0 扱いに
            # するので、値が 0 でも送り続けるのが正しい
            rec = DriveCommand(timestamp=time.time(),
                               forward=drive["forward"], yaw=drive["yaw"],
                               height=drive["height"])
            node.send_output("drive_command",
                             pa.array(list(pack_drive_command(rec)), type=pa.uint8()))
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        stop_ev.set()


if __name__ == "__main__":
    main()
