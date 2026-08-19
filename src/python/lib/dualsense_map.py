"""DualSense (PS5) の joystick API ボタン番号を、ドライバに依らず物理ボタン名で引く。

背景: 同じ DualSense でも、掴む HID ドライバで js のボタン番号が変わる。

  * hid-generic (ロボットの JetPack RT カーネル。CONFIG_HID_PLAYSTATION 無し):
      意味を知らないので HID usage の並び順に BTN_GAMEPAD(0x130)+n を機械的に振る
      → js 番号 = usage-1 = □0 ×1 ○2 △3 L1 4 R1 5 L2 6 R2 7 Create 8 Options 9 L3 10 R3 11 PS 12
  * hid_playstation (一般的なデスクトップ Linux):
      物理的な意味どおりのキーコードを振る (× = BTN_SOUTH, ○ = BTN_EAST, □ = BTN_WEST, …)
      → js 番号 = ×0 ○1 △2 □3 L1 4 R1 5 L2 6 R2 7 Create 8 Options 9 PS 10 L3 11 R3 12

キーコード名 (BTN_SOUTH 等) だけでは足りない: hid-generic の BTN_SOUTH は □、
hid_playstation の BTN_SOUTH は × を指す。なので「どのドライバか」を先に判定し、
ドライバごとの {キーコード → 物理名} 表を JSIOCGBTNMAP (js 番号 → キーコード) に
当てて {物理名 → js 番号} を得る。

判定: sysfs の /sys/class/input/jsN/device/device/driver の実体名 (playstation /
hid-generic)。読めない場合は JSIOCGBTNMAP の中身で推定 (BTN_C 0x132 があれば generic、
BTN_THUMBR 0x13e があれば playstation)。環境変数 DUALSENSE_SCHEME=generic|playstation で
強制もできる。

軸は両ドライバとも左スティック = ABS_X/ABS_Y なので JSIOCGAXMAP から引く (結果は 0/1)。

stdlib のみ。dualsense_input (dora ノード) と scripts/dualsense_monitor から使う。
"""

from __future__ import annotations

import array
import fcntl
import os
from dataclasses import dataclass, field

# <linux/joystick.h>
JSIOCGAXES = 0x80016A11      # _IOR('j', 0x11, __u8)
JSIOCGBUTTONS = 0x80016A12   # _IOR('j', 0x12, __u8)
JSIOCGAXMAP = 0x80406A32     # _IOR('j', 0x32, __u8[ABS_CNT=0x40])
JSIOCGBTNMAP = 0x84026A34    # _IOR('j', 0x34, __u16[KEY_MAX - BTN_MISC + 1 = 513])
_BTNMAP_LEN = 513
_AXMAP_LEN = 0x40

# <linux/input-event-codes.h>
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
ABS_HAT0X, ABS_HAT0Y = 0x10, 0x11
BTN_GAMEPAD = 0x130
BTN_SOUTH, BTN_EAST, BTN_C, BTN_NORTH, BTN_WEST, BTN_Z = 0x130, 0x131, 0x132, 0x133, 0x134, 0x135
BTN_TL, BTN_TR, BTN_TL2, BTN_TR2 = 0x136, 0x137, 0x138, 0x139
BTN_SELECT, BTN_START, BTN_MODE, BTN_THUMBL, BTN_THUMBR = 0x13A, 0x13B, 0x13C, 0x13D, 0x13E

# 物理ボタン名 (このモジュールの語彙)。表示用の記号は GLYPH。
SQUARE, CROSS, CIRCLE, TRIANGLE = "square", "cross", "circle", "triangle"
L1, R1, L2, R2 = "L1", "R1", "L2", "R2"
CREATE, OPTIONS, PS, L3, R3, TOUCHPAD = "create", "options", "PS", "L3", "R3", "touchpad"
GLYPH = {SQUARE: "□", CROSS: "×", CIRCLE: "○", TRIANGLE: "△",
         CREATE: "Create", OPTIONS: "Options"}

# hid_playstation: 意味どおりのキーコード
_KEYCODE_TO_NAME_PLAYSTATION = {
    BTN_SOUTH: CROSS, BTN_EAST: CIRCLE, BTN_NORTH: TRIANGLE, BTN_WEST: SQUARE,
    BTN_TL: L1, BTN_TR: R1, BTN_TL2: L2, BTN_TR2: R2,
    BTN_SELECT: CREATE, BTN_START: OPTIONS, BTN_MODE: PS,
    BTN_THUMBL: L3, BTN_THUMBR: R3,
}
# hid-generic: DualSense の HID ディスクリプタの usage 順に BTN_GAMEPAD+n
_GENERIC_USAGE_ORDER = (SQUARE, CROSS, CIRCLE, TRIANGLE, L1, R1, L2, R2,
                        CREATE, OPTIONS, L3, R3, PS, TOUCHPAD)
_KEYCODE_TO_NAME_GENERIC = {BTN_GAMEPAD + i: n for i, n in enumerate(_GENERIC_USAGE_ORDER)}

SCHEMES = {"generic": _KEYCODE_TO_NAME_GENERIC, "playstation": _KEYCODE_TO_NAME_PLAYSTATION}

# 軸の役割名と、スキームごとの {ABS コード → 役割}。
# 左スティックは両者 ABS_X/ABS_Y だが、右スティックとトリガは割り当てが違う:
#   hid-generic:     HID usage Z/Rz が右スティック、Rx/Ry がトリガ (ディスクリプタの並びそのまま)
#   hid_playstation: 右スティック = ABS_RX/ABS_RY、L2/R2 = ABS_Z/ABS_RZ (意味どおり)
# joydev の js 軸番号は ABS コード昇順なので、どちらも js 0..7 = X Y Z RX RY RZ HAT0X HAT0Y。
# → generic: 右スティック js 2/5, L2 js 3, R2 js 4 / playstation: 右スティック js 3/4, L2 js 2, R2 js 5
AX_LX, AX_LY, AX_RX, AX_RY, AX_L2, AX_R2, AX_DX, AX_DY = "LX", "LY", "RX", "RY", "L2", "R2", "DX", "DY"
_AXIS_ROLES = {
    "generic": {ABS_X: AX_LX, ABS_Y: AX_LY, ABS_Z: AX_RX, ABS_RZ: AX_RY,
                ABS_RX: AX_L2, ABS_RY: AX_R2, ABS_HAT0X: AX_DX, ABS_HAT0Y: AX_DY},
    "playstation": {ABS_X: AX_LX, ABS_Y: AX_LY, ABS_RX: AX_RX, ABS_RY: AX_RY,
                    ABS_Z: AX_L2, ABS_RZ: AX_R2, ABS_HAT0X: AX_DX, ABS_HAT0Y: AX_DY},
}


def glyph(name: str) -> str:
    return GLYPH.get(name, name)


def detect_driver(dev: str) -> str | None:
    """/dev/input/jsN → それを生やした HID ドライバ名 ('playstation' / 'hid-generic' / None)"""
    js = os.path.basename(dev)
    try:
        return os.path.basename(os.readlink(f"/sys/class/input/{js}/device/device/driver"))
    except OSError:
        return None


def _scheme_for(driver: str | None, btnmap: list[int]) -> tuple[str, str]:
    """(scheme, 根拠) を返す"""
    forced = os.environ.get("DUALSENSE_SCHEME")
    if forced:
        if forced not in SCHEMES:
            raise ValueError(f"DUALSENSE_SCHEME={forced!r}: choose from {sorted(SCHEMES)}")
        return forced, "env DUALSENSE_SCHEME"
    if driver == "playstation":
        return "playstation", "sysfs driver=playstation"
    if driver == "hid-generic":
        return "generic", "sysfs driver=hid-generic"
    # フォールバック: キーコードの顔ぶれで推定
    codes = set(btnmap)
    if BTN_C in codes:
        return "generic", "btnmap has BTN_C (0x132)"
    if BTN_THUMBR in codes and BTN_C not in codes:
        return "playstation", "btnmap has BTN_THUMBR (0x13e), no BTN_C"
    return "generic", f"unknown driver {driver!r}; assuming robot default (hid-generic)"


@dataclass
class DualSenseMap:
    driver: str | None
    scheme: str                        # "generic" | "playstation"
    reason: str                        # どうやって scheme を決めたか
    index_of: dict[str, int]           # 物理名 → js ボタン番号
    name_of: dict[int, str]            # js ボタン番号 → 物理名
    axis_index_of: dict[str, int] = field(default_factory=dict)   # 軸の役割 (AX_*) → js 軸番号
    axis_role_of: dict[int, str] = field(default_factory=dict)    # js 軸番号 → 役割
    axis_lx: int = 0
    axis_ly: int = 1
    num_axes: int = 0
    num_buttons: int = 0
    keycodes: list[int] = field(default_factory=list)   # js 番号順のキーコード (デバッグ用)
    abscodes: list[int] = field(default_factory=list)   # js 軸番号順の ABS コード (デバッグ用)

    @classmethod
    def from_fd(cls, fd: int, dev: str = "/dev/input/js0") -> "DualSenseMap":
        naxes = array.array("B", [0]);  fcntl.ioctl(fd, JSIOCGAXES, naxes)
        nbtn = array.array("B", [0]);   fcntl.ioctl(fd, JSIOCGBUTTONS, nbtn)
        bm = array.array("H", [0] * _BTNMAP_LEN); fcntl.ioctl(fd, JSIOCGBTNMAP, bm)
        am = array.array("B", [0] * _AXMAP_LEN);  fcntl.ioctl(fd, JSIOCGAXMAP, am)
        return cls.from_maps(list(bm[: nbtn[0]]), list(am[: naxes[0]]), detect_driver(dev))

    @classmethod
    def from_maps(cls, keycodes: list[int], axes: list[int], driver: str | None) -> "DualSenseMap":
        """JSIOCGBTNMAP / JSIOCGAXMAP の中身とドライバ名から組み立てる (テストもここを使う)"""
        scheme, reason = _scheme_for(driver, keycodes)
        if scheme == "generic":
            # hid-generic は usage 順に連番のキーコードを振り、joydev はキーコード昇順に
            # js 番号を振るので「js 番号 = usage 順」。基点 (BTN_GAMEPAD / BTN_JOYSTICK) に
            # 依らないよう番号順で引く (= README の実測番号そのもの)
            name_of = {i: n for i, n in enumerate(_GENERIC_USAGE_ORDER) if i < len(keycodes)}
        else:
            table = SCHEMES[scheme]
            name_of = {i: table[c] for i, c in enumerate(keycodes) if c in table}
        index_of = {n: i for i, n in name_of.items()}
        roles = _AXIS_ROLES[scheme]
        axis_role_of = {i: roles[c] for i, c in enumerate(axes) if c in roles}
        axis_index_of = {r: i for i, r in axis_role_of.items()}
        return cls(
            driver=driver, scheme=scheme, reason=reason,
            index_of=index_of, name_of=name_of,
            axis_index_of=axis_index_of, axis_role_of=axis_role_of,
            axis_lx=axis_index_of.get(AX_LX, 0), axis_ly=axis_index_of.get(AX_LY, 1),
            num_axes=len(axes), num_buttons=len(keycodes), keycodes=keycodes, abscodes=axes,
        )

    @classmethod
    def default_generic(cls) -> "DualSenseMap":
        """ioctl が使えないとき (js でないデバイス等) のロボット既定 = hid-generic 番号"""
        return cls.from_maps([BTN_GAMEPAD + i for i in range(len(_GENERIC_USAGE_ORDER))],
                             [ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_HAT0X, ABS_HAT0Y],
                             "hid-generic")._replace_reason("fallback (ioctl failed); assuming hid-generic")

    def _replace_reason(self, reason: str) -> "DualSenseMap":
        self.reason = reason
        self.driver = None
        return self

    def describe(self) -> str:
        order = sorted(self.name_of.items())
        axes = " ".join(f"{r}={i}" for i, r in sorted(self.axis_role_of.items()))
        return (f"driver={self.driver} scheme={self.scheme} ({self.reason}); "
                + " ".join(f"{glyph(n)}={i}" for i, n in order)
                + f"; axes: {axes}")
