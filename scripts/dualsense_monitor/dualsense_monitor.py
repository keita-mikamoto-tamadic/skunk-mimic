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

割り当ての根拠は README.md (hid-generic での実測 2026-08-18)。

ボタン/軸の番号は掴んでいる HID ドライバで変わる (ロボットの hid-generic: □0 ×1 ○2 △3 …
L3 10、右スティック axis 2/5、L2/R2 axis 3/4 / PC の hid_playstation: ×0 ○1 △2 □3 … L3 11、
右スティック axis 3/4、L2/R2 axis 2/5)。番号はハードコードせず、接続のたびに
src/python/lib/dualsense_map (stdlib のみ) が sysfs のドライバ名 + JSIOCGBTNMAP/AXMAP から
{物理名 → 番号} を解決する。画面の 1 行目に driver/scheme が出る。
どのドライバかは `readlink /sys/class/input/js0/device/device/driver` でも分かる。
"""

import os
import select
import struct
import sys
import time

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "python"))
from lib import dualsense_map as dsm  # noqa: E402  (stdlib のみ、uv 不要)

# struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
JS_FMT = "<IhBB"
JS_SIZE = 8
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

AXIS_MAX = 32767
RENDER_HZ = 30
EVENT_LOG_LINES = 6

# 表示する役割 (番号は接続時に st.map から引く)
# スティック: (表示名, 役割 左右, 役割 上下)
STICKS = (("左", dsm.AX_LX, dsm.AX_LY), ("右", dsm.AX_RX, dsm.AX_RY))
# トリガ: 中立が -32767 なので 0..1 に正規化する
TRIGGERS = (("L2", dsm.AX_L2), ("R2", dsm.AX_R2))
# ボタンの表示順 (3 行)
BUTTON_ROWS = (
    (dsm.SQUARE, dsm.CROSS, dsm.CIRCLE, dsm.TRIANGLE),
    (dsm.L1, dsm.R1, dsm.L2, dsm.R2),
    (dsm.CREATE, dsm.OPTIONS, dsm.L3, dsm.R3, dsm.PS),
)


def resolve_map(f, dev):
    try:
        return dsm.DualSenseMap.from_fd(f.fileno(), dev)
    except OSError:
        return dsm.DualSenseMap.default_generic()


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
        self.map = dsm.DualSenseMap.default_generic()   # 接続時に実デバイスのものへ差し替え

    def set_map(self, m):
        self.map = m
        self.axes.clear()
        self.buttons.clear()

    def apply(self, value, etype, number):
        init = bool(etype & JS_EVENT_INIT)
        if etype & JS_EVENT_BUTTON:
            self.buttons[number] = value
            if not init:
                name = dsm.glyph(self.map.name_of.get(number, "?"))
                self.log(f"BTN  {number:<2} {name:<7} {'押した' if value else '離した'}")
        elif etype & JS_EVENT_AXIS:
            self.axes[number] = value
            # 軸は動かすと大量に来るのでログには残さない (未知の軸だけ記録)
            if not init and number not in self.map.axis_role_of:
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
    m = st.map
    out.append(f"  driver={m.driver}  scheme={m.scheme}  ({m.reason})  "
               f"buttons={m.num_buttons} axes={m.num_axes}")
    out.append("")
    ax = m.axis_index_of   # 役割 → js 軸番号 (無ければ -1 表示)

    out.append("── スティック ──────────────────────────────────────")
    for label, rx, ry in STICKS:
        ix, iy = ax.get(rx, -1), ax.get(ry, -1)
        x = norm_stick(st.axes.get(ix, 0))
        y = norm_stick(st.axes.get(iy, 0))
        out.append(f"  {label} (axis {ix}/{iy})")
        out.append(f"      左右 {x:+5.2f} [{bar_signed(x)}]")
        out.append(f"      上下 {y:+5.2f} [{bar_signed(y)}]  (上が負)")
    out.append("")

    out.append("── トリガ (中立 -32767) ────────────────────────────")
    cells = []
    for label, role in TRIGGERS:
        i = ax.get(role, -1)
        t = norm_trigger(st.axes.get(i, -AXIS_MAX))
        cells.append(f"{label} (axis {i}) {t:4.2f} [{bar_unsigned(t)}]")
    out.append("  " + "    ".join(cells))
    out.append("")

    idx, idy = ax.get(dsm.AX_DX, -1), ax.get(dsm.AX_DY, -1)
    dx = st.axes.get(idx, 0)
    dy = st.axes.get(idy, 0)
    out.append("── 十字キー ────────────────────────────────────────")
    out.append(f"  axis {idx}/{idy} = {dx:+6d} / {dy:+6d}      {dpad_arrow(dx, dy)}")
    out.append("")

    out.append("── ボタン (● = 押下) ───────────────────────────────")
    for row in BUTTON_ROWS:
        cells = []
        for name in row:
            n = m.index_of.get(name)
            if n is None:
                cells.append(f"{dsm.glyph(name):>7} -- ・")
                continue
            mark = "●" if st.buttons.get(n) else "・"
            cells.append(f"{dsm.glyph(name):>7} {n:<2} {mark}")
        out.append("  " + "  ".join(cells))
    unknown = sorted(n for n in st.buttons if n not in m.name_of)
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
                if f is not None:
                    st.set_map(resolve_map(f, dev))
                    st.log(f"接続: {st.map.describe()}"[:110])
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
