"""DualSense (PS5) で state_command を送る dora ノード。dummy_input のパッド版。

robot_web_gui と同じく RCM の状態機械を経由する (motor_commands を直接送る
web_controller とは別物)。入力トピックは持たない (dummy_input と同じ、出力のみ)。

## 設計上の約束

**BT 接続をノードの起動条件にしない。** パッドが繋がっていなくても起動し、
実行中に切れても落ちない。dynamic ノードは全 attach まで dataflow を開始バリアで
待たせるため、ここで落ちると dataflow ごと止まる。

**切断時は何も送らない。** RCM の状態はそのまま維持する (勝手に SERVO_OFF /
STOP を送らない)。将来 走行指令 (前後・旋回) を追加したら、そちらは切断時に
0 へ戻すこと — 押しっぱなしの値が残ると暴走する。

**送信はメインスレッドのみ。** パッドの読み取りは別スレッドでキューに積み、
dora のイベントループ側で drain して送る (robot_web_gui と同じ方式)。

## 割り当て

  ○  (2)                SERVO_ON              OFF のみ
  △  (3)                READY                 STOP のみ
  □  (0)                RUN                   READY + 補間完了のみ
  ×  (1)                STOP                  OFF 以外
  L1+L3 (4+10)          SERVO_OFF             常時 (緊急停止)
  Create+Options (8+9)  INIT_POSITION_RESET   OFF のみ (ストッパー押し当て後に!)

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

# struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
JS_FMT = "<IhBB"
JS_SIZE = 8
JS_EVENT_BUTTON = 0x01
JS_EVENT_INIT = 0x80

DEV = os.environ.get("DUALSENSE_DEV", "/dev/input/js0")
REOPEN_INTERVAL = 1.0     # 切断中の再オープン試行間隔 [s]
LOOP_TIMEOUT = 0.05       # dora イベント待ちのタイムアウト [s] = drain 周期

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

def log(msg):
    print(f"[dualsense_input] {msg}", flush=True)


def reader_thread(dev, out_q, stop_ev):
    """js0 を読んで発火した (StateCommand, 名前) をキューに積む。

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
        # 切断時は state_command を送らない (RCM の状態を維持)。
        # 走行指令を足したら、ここでその入力を 0 に戻すこと。
        log(f"disconnected ({reason}) — 状態は維持。再接続を待つ")

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
        if etype & JS_EVENT_INIT or not (etype & JS_EVENT_BUTTON):
            # INIT はオープン直後の状態通知。軸は今は使わない
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
    stop_ev = threading.Event()
    threading.Thread(target=reader_thread, args=(DEV, out_q, stop_ev),
                     daemon=True).start()

    log(f"device={DEV} (未接続でも起動する / 切断しても落ちない)")
    for b, (_, name) in sorted(SINGLE.items()):
        log(f"  {btn_name(b):<15} -> {name}")
    for (a, b), (_, name) in COMBO.items():
        log(f"  {btn_name(a)}+{btn_name(b):<12} -> {name}")

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
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        stop_ev.set()


if __name__ == "__main__":
    main()
