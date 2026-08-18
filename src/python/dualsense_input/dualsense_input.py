"""DualSense (PS5) で state_command を送る dora ノード。dummy_input のパッド版。

robot_web_gui と同じく RCM の状態機械を経由する (motor_commands を直接送る
web_controller とは別物)。入力トピックは持たない (出力のみ)。

state_command (ボタン、イベント駆動) に加えて drive_command (左スティック上下 →
前後の走行指令) を約 50Hz で送り続ける。drive_command は stabilizer の外側ループの
目標ホイール速度に入る (詳細は command_data.json の doc)。

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

## 割り当て

  ○  (2)                SERVO_ON              OFF のみ
  △  (3)                READY                 STOP のみ
  □  (0)                RUN                   READY + 補間完了のみ
  ×  (1)                STOP                  OFF 以外
  L1+L3 (4+10)          SERVO_OFF             常時 (緊急停止)
  Create+Options (8+9)  INIT_POSITION_RESET   OFF のみ (ストッパー押し当て後に!)
  左スティック上下       drive_command.forward 上=前進。正規化 -1..1、デッドゾーン 0.05
  左スティック左右       drive_command.yaw     右=右旋回。正規化 -1..1、デッドゾーン 0.05
                        (何 rad/s になるかは制御側 kMaxDriveWheelVel / kMaxYawWheelDiff)

右の条件は RCM 側のガード (robot_control_manager.cpp の HandleStateCommand)。
ここでは弾かずに送るだけで、通らなければ RCM が黙って無視する。

同時押しは「両方が押されている状態で、後から押した方の押下イベント」で 1 回だけ
発火する。同時押しに使うボタンは単押しに割り当てていないので誤爆しない。

割り当ての実測根拠は scripts/dualsense_monitor/README.md。hid-playstation を
入れると番号が変わるので、その時は両方を直すこと。

## 使い方

  cd src/python && uv run dualsense_input/dualsense_input.py

  DUALSENSE_DEV   デバイス (既定 /dev/input/js0)
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
AXIS_LY = 1               # 左スティック上下 (実測: 上が負) → forward
AXIS_LX = 0               # 左スティック左右 (実測: 右が正) → yaw
STICK_DEADZONE = 0.05     # 正規化後のデッドゾーン

BTN_SQUARE, BTN_CROSS, BTN_CIRCLE, BTN_TRIANGLE = 0, 1, 2, 3
BTN_L1, BTN_R1 = 4, 5
BTN_CREATE, BTN_OPTIONS = 8, 9
BTN_L3, BTN_R3 = 10, 11

BTN_NAMES = {
    BTN_SQUARE: "□", BTN_CROSS: "×", BTN_CIRCLE: "○", BTN_TRIANGLE: "△",
    BTN_L1: "L1", BTN_R1: "R1",
    BTN_CREATE: "Create", BTN_OPTIONS: "Options",
    BTN_L3: "L3", BTN_R3: "R3",
}

# 単押し: {button: (StateCommand, 表示名)}
SINGLE = {
    BTN_CIRCLE:   (StateCommand.SERVO_ON, "SERVO_ON"),
    BTN_TRIANGLE: (StateCommand.READY,    "READY"),
    BTN_SQUARE:   (StateCommand.RUN,      "RUN"),
    BTN_CROSS:    (StateCommand.STOP,     "STOP"),
}
# 同時押し: {(button, button): (StateCommand, 表示名)}
COMBO = {
    (BTN_L1, BTN_L3):          (StateCommand.SERVO_OFF, "SERVO_OFF"),
    (BTN_CREATE, BTN_OPTIONS): (StateCommand.INIT_POSITION_RESET, "INIT_POSITION_RESET"),
}
COMBO_MEMBERS = {b for pair in COMBO for b in pair}

def btn_name(n):
    return BTN_NAMES.get(n, f"BTN{n}")


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
    pressed = set()
    next_try = 0.0

    def close(reason):
        nonlocal f
        if f is not None:
            try:
                f.close()
            except OSError:
                pass
        f = None
        pressed.clear()
        # 切断時: state_command は送らない (RCM の状態を維持) が、
        # 走行指令は 0 に戻す (倒しっぱなしの値を残さない)
        drive["forward"] = 0.0
        drive["yaw"] = 0.0
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
            log(f"connected: {dev}")

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
        if etype & JS_EVENT_AXIS and number == AXIS_LY:
            # 軸は INIT (オープン直後の現在値) も受ける — 実際の位置なので
            drive["forward"] = stick_to_norm(value, invert=True)  # 上=前進=正
            continue
        if etype & JS_EVENT_AXIS and number == AXIS_LX:
            drive["yaw"] = stick_to_norm(value)                   # 右=右旋回=正
            continue
        if etype & JS_EVENT_INIT or not (etype & JS_EVENT_BUTTON):
            # ボタンの INIT は無視 (押下イベントでのみ発火させる)。他の軸も未使用
            continue
        if not value:
            pressed.discard(number)
            continue

        pressed.add(number)
        fired = False
        # 同時押しを先に判定。今押したボタンが組の一方で、もう一方も押下中なら発火
        for (a, b), entry in COMBO.items():
            other = b if number == a else (a if number == b else None)
            if other is not None and other in pressed:
                out_q.put(entry)
                fired = True
        if fired:
            continue
        if number in SINGLE and number not in COMBO_MEMBERS:
            out_q.put(SINGLE[number])


def main():
    node = Node("dualsense_input")
    out_q = queue.Queue()
    drive = {"forward": 0.0, "yaw": 0.0}   # reader が書き、main が送る (GIL で atomic)
    stop_ev = threading.Event()
    threading.Thread(target=reader_thread, args=(DEV, out_q, drive, stop_ev),
                     daemon=True).start()

    log(f"device={DEV} (未接続でも起動する / 切断しても落ちない)")
    for b, (_, name) in sorted(SINGLE.items()):
        log(f"  {btn_name(b):<15} -> {name}")
    for (a, b), (_, name) in COMBO.items():
        log(f"  {btn_name(a)}+{btn_name(b):<12} -> {name}")
    log(f"  左スティック上下  -> drive_command.forward (上=前進, 正規化 -1..1)")
    log(f"  左スティック左右  -> drive_command.yaw     (右=右旋回, 正規化 -1..1)")
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
                               forward=drive["forward"], yaw=drive["yaw"])
            node.send_output("drive_command",
                             pa.array(list(pack_drive_command(rec)), type=pa.uint8()))
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        stop_ev.set()


if __name__ == "__main__":
    main()
