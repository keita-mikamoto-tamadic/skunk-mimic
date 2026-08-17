#!/usr/bin/env python3
"""DualSense (PS5) の入力を端末に表示するモニタ。dora ノードではない。

Linux の joystick API (/dev/input/js*) を直接読む。hid-playstation ドライバが
無いカーネル (JetPack 7 / L4T r39.2 の RT カーネルは CONFIG_HID_PLAYSTATION 未設定)
でも、hid-generic + joydev が js0 を生やすのでこれで読める。

  * stdlib のみ (uv 不要、`python3 dualsense_monitor.py` で動く)
  * /dev/input/js0 は crw-rw-r-- なので一般ユーザーで読める (udev ルール不要)
  * 抜き差しに追従する (切断したら再オープンを待つ)

用途は 2 つ:
  1. コントローラが生きているか / どのボタンが効くかの確認
  2. 未確定の割り当て (BTN 10/11/13/14 等) を実測で埋める ← 下部のイベントログ

使い方:
  python3 scripts/dualsense_monitor/dualsense_monitor.py [デバイス]
  DUALSENSE_DEV=/dev/input/js1 python3 scripts/dualsense_monitor/dualsense_monitor.py

割り当ての根拠は README.md。実測 (2026-08-18) で確定した対応を持っている。
"""

import os
import select
import struct
import sys
import time

# struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
JS_FMT = "<IhBB"
JS_SIZE = 8
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

AXIS_MAX = 32767
RENDER_HZ = 30
EVENT_LOG_LINES = 6

# 実測で確定した対応 (README.md 参照)
BUTTON_NAMES = {
    0: "□", 1: "×", 2: "○", 3: "△",
    4: "L1", 5: "R1", 6: "L2", 7: "R2",
    8: "Create", 9: "Options", 12: "PS",
}
# スティック: (表示名, axis 左右, axis 上下)
STICKS = (("左", 0, 1), ("右", 2, 5))
# トリガ: 中立が -32767 なので 0..1 に正規化する
TRIGGERS = (("L2", 3), ("R2", 4))
DPAD_X, DPAD_Y = 6, 7


def norm_stick(v):
    """-32767..32767 → -1.0..1.0"""
    return max(-1.0, min(1.0, v / AXIS_MAX))


def norm_trigger(v):
    """-32767 (離す)..32767 (全押し) → 0.0..1.0"""
    return max(0.0, min(1.0, (v + AXIS_MAX) / (2.0 * AXIS_MAX)))


def bar_signed(x, width=21):
    """-1..1 を中央 0 のバーにする。"""
    half = width // 2
    n = int(round(abs(x) * half))
    if x >= 0:
        return " " * half + "|" + "=" * n + " " * (half - n)
    return " " * (half - n) + "=" * n + "|" + " " * half


def bar_unsigned(x, width=12):
    n = int(round(x * width))
    return "=" * n + " " * (width - n)


def dpad_arrow(x, y):
    """十字キーの状態を矢印にする (上が負)。"""
    v = ""
    if y < 0:
        v += "↑"
    elif y > 0:
        v += "↓"
    if x < 0:
        v += "←"
    elif x > 0:
        v += "→"
    return v if v else "・"


class State:
    def __init__(self):
        self.axes = {}
        self.buttons = {}
        self.events = []          # 直近のイベント (新しいものが先頭)
        self.total = 0
        self.connected = False
        self.last_rx = 0.0

    def apply(self, value, etype, number):
        init = bool(etype & JS_EVENT_INIT)
        if etype & JS_EVENT_BUTTON:
            self.buttons[number] = value
            if not init:
                name = BUTTON_NAMES.get(number, "?")
                self.log(f"BTN  {number:<2} {name:<7} {'押した' if value else '離した'}")
        elif etype & JS_EVENT_AXIS:
            self.axes[number] = value
            # 軸は動かすと大量に来るのでログには残さない (未知の軸だけ記録)
            if not init and number not in (0, 1, 2, 5, 3, 4, DPAD_X, DPAD_Y):
                self.log(f"AXIS {number:<2} {'(未割当)':<7} {value:+6d}")
        self.total += 1
        self.last_rx = time.monotonic()

    def log(self, line):
        self.events.insert(0, (time.strftime("%H:%M:%S"), line))
        del self.events[EVENT_LOG_LINES:]


def render(st, dev):
    out = []
    status = "接続中" if st.connected else "切断 (再接続待ち)"
    age = time.monotonic() - st.last_rx if st.last_rx else -1.0
    idle = f"{age:5.1f}s 無操作" if age >= 0 else "  --  "
    out.append(f"DualSense monitor   {dev}   [{status}]   {idle}   events={st.total}")
    out.append("")

    out.append("── スティック ──────────────────────────────────────")
    for label, ax_x, ax_y in STICKS:
        x = norm_stick(st.axes.get(ax_x, 0))
        y = norm_stick(st.axes.get(ax_y, 0))
        out.append(f"  {label} (axis {ax_x}/{ax_y})")
        out.append(f"      左右 {x:+5.2f} [{bar_signed(x)}]")
        out.append(f"      上下 {y:+5.2f} [{bar_signed(y)}]  (上が負)")
    out.append("")

    out.append("── トリガ (中立 -32767) ────────────────────────────")
    cells = []
    for label, ax in TRIGGERS:
        t = norm_trigger(st.axes.get(ax, -AXIS_MAX))
        cells.append(f"{label} (axis {ax}) {t:4.2f} [{bar_unsigned(t)}]")
    out.append("  " + "    ".join(cells))
    out.append("")

    dx = st.axes.get(DPAD_X, 0)
    dy = st.axes.get(DPAD_Y, 0)
    out.append("── 十字キー ────────────────────────────────────────")
    out.append(f"  axis {DPAD_X}/{DPAD_Y} = {dx:+6d} / {dy:+6d}      {dpad_arrow(dx, dy)}")
    out.append("")

    out.append("── ボタン (● = 押下) ───────────────────────────────")
    known = [(n, BUTTON_NAMES[n]) for n in sorted(BUTTON_NAMES)]
    for row in (known[0:4], known[4:8], known[8:]):
        cells = []
        for n, name in row:
            mark = "●" if st.buttons.get(n) else "・"
            cells.append(f"{name:>7} {n:<2} {mark}")
        out.append("  " + "  ".join(cells))
    unknown = sorted(n for n in st.buttons if n not in BUTTON_NAMES)
    if unknown:
        cells = [f"{n:<2} {'●' if st.buttons[n] else '・'}" for n in unknown]
        out.append("  未割当: " + "  ".join(cells))
    out.append("")

    out.append(f"── 直近のイベント (軸は未割当のみ記録) ─────────────")
    for ts, line in st.events:
        out.append(f"  {ts}  {line}")
    for _ in range(EVENT_LOG_LINES - len(st.events)):
        out.append("")
    out.append("")
    out.append("Ctrl-C で終了")
    return "\n".join(out)


def open_device(path):
    try:
        return open(path, "rb")
    except OSError:
        return None


def main():
    dev = (sys.argv[1] if len(sys.argv) > 1
           else os.environ.get("DUALSENSE_DEV", "/dev/input/js0"))
    st = State()
    f = None
    period = 1.0 / RENDER_HZ
    next_draw = 0.0

    sys.stdout.write("\033[?25l")           # カーソル非表示
    try:
        while True:
            if f is None:
                f = open_device(dev)
                st.connected = f is not None
                if f is None:
                    # 未接続でも画面は出し続ける (抜き差し待ち)
                    sys.stdout.write("\033[H\033[J" + render(st, dev) + "\n")
                    sys.stdout.flush()
                    time.sleep(0.5)
                    continue

            if select.select([f], [], [], period)[0]:
                data = f.read(JS_SIZE)
                if not data or len(data) < JS_SIZE:
                    # 抜かれた
                    f.close()
                    f = None
                    st.connected = False
                    st.log("デバイスが切断された")
                    continue
                _, value, etype, number = struct.unpack(JS_FMT, data)
                st.apply(value, etype, number)

            now = time.monotonic()
            if now >= next_draw:
                next_draw = now + period
                sys.stdout.write("\033[H\033[J" + render(st, dev) + "\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    except OSError as e:
        sys.stdout.write("\033[?25h")
        sys.exit(f"dualsense_monitor: {dev}: {e}")
    finally:
        sys.stdout.write("\033[?25h")       # カーソル復帰
        sys.stdout.flush()


if __name__ == "__main__":
    main()
