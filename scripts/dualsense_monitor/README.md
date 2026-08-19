# dualsense_monitor

DualSense (PS5) の入力を端末に表示するモニタ。**dora ノードではない**ので、
dataflow の起動状態に関係なく好きなときに立てて Ctrl-C で閉じられる。

```bash
python3 scripts/dualsense_monitor/dualsense_monitor.py
python3 scripts/dualsense_monitor/dualsense_monitor.py /dev/input/js1   # デバイス指定
DUALSENSE_DEV=/dev/input/js1 python3 scripts/dualsense_monitor/dualsense_monitor.py
```

**stdlib のみ。`uv` は不要** (`scripts/pyproject.toml` の workspace メンバにも入れていない)。
`/dev/input/js0` は `crw-rw-r--` なので一般ユーザーで読める = udev ルールも不要。

抜き差しに追従する。切断すると「再接続待ち」表示のまま待ち、挿し直すと復帰する。

## 前提: このカーネルでは hid-playstation が無い

JetPack 7.2.1 / L4T r39.2 の RT カーネル (`6.8.12-1021-rt-tegra`) は
**`CONFIG_HID_PLAYSTATION` も `CONFIG_HID_SONY` も未設定**で、専用ドライバが無い。

それでも `hid-generic` が HID デスクリプタを解釈し、`CONFIG_INPUT_JOYDEV=y` によって
`/dev/input/js0` と `event5` が生える。よって **hidraw を自前でパースする必要はなく、
標準の joystick API で読める**。

**USB と Bluetooth の両方で動く。軸・ボタン番号も両者で同一** (どちらも実機確認済み)。

```
# USB
Bus 001 Device 005: ID 054c:0ce6 Sony Corp. DualSense wireless controller (PS5)
Port 002: Dev 005, If 3, Class=Human Interface Device, Driver=usbhid
I: Bus=0003 ...  N: Name="Sony Interactive Entertainment DualSense Wireless Controller"
H: Handlers=js0 event5

# Bluetooth
I: Bus=0005 Vendor=054c Product=0ce6 Version=0100
N: Name="DualSense Wireless Controller"
H: Handlers=js0 event5
```

USB では Interface 0〜2 が DualSense 内蔵オーディオ (`snd-usb-audio`)、ゲームパッドは If 3。

## Bluetooth 接続手順

**USB ケーブルを抜いてから**行う (USB 接続中はペアリングモードに入らない)。

1. コントローラ側: **Create ボタン + PS ボタンを同時に長押し** → ライトバーが素早く点滅
2. Jetson 側:

```bash
bluetoothctl pairable on        # Pairable: no の場合のみ
bluetoothctl
  scan on                      # "DualSense Wireless Controller" が出るまで待つ
  scan off
  pair    <MAC>
  trust   <MAC>                # 以後 PS ボタンだけで自動再接続
  connect <MAC>
  exit
```

- **一般ユーザーで可**。`/usr/share/dbus-1/system.d/bluetooth.conf` の
  `<policy context="default">` が `send_destination="org.bluez"` を許可しているため、
  `bluetooth` グループに入る必要はない
- 確認: `bluetoothctl info` で `Paired/Trusted/Connected: yes`、
  `UUID: Human Interface Device` が出ていること
- **USB と BT を同時に繋ぐと `js0` と `js1` の両方が生え、どちらがどちらか起動順次第**。
  その場合は本モニタに引数でデバイスを明示する

## 実測した割り当て (2026-08-18, USB / Bluetooth 共通)

hid-generic + joydev による汎用マッピングなので、`hid-playstation` を入れると
**番号が変わる** (PC で実測 2026-08-20: ×0 ○1 △2 □3 L1 4 R1 5 L2 6 R2 7 Create 8 Options 9
PS 10 L3 11 R3 12、右スティック axis 3/4、L2/R2 axis 2/5、十字キー 6/7)。

このモニタと `dualsense_input` は番号をハードコードせず、接続時に
`src/python/lib/dualsense_map.py` が sysfs のドライバ名 (`/sys/class/input/js0/device/device/driver`
→ `playstation` / `hid-generic`) と `JSIOCGBTNMAP` / `JSIOCGAXMAP` から {物理名 → 番号} を
解決するので、どちらのドライバでも正しい名前で出る (画面 1 行目に driver/scheme 表示)。
下の表は **hid-generic (ロボット) の番号**で、dualsense_map の generic 表の根拠。

### ボタン

| js button | DualSense | | js button | DualSense |
|---|---|---|---|---|
| 0 | □ | | 6 | L2 (デジタル) |
| 1 | × | | 7 | R2 (デジタル) |
| 2 | ○ | | 8 | Create |
| 3 | △ | | 9 | Options |
| 4 | L1 | | 12 | PS |
| 5 | R1 | | 10/11/13/14 | 未確定 |

`□ × ○ △` = `0 1 2 3` は HID レポートのビット 0〜3 の順そのまま。
未確定分は本モニタ下部の「直近のイベント」に生の番号が出るので、押して埋められる。

### 軸

| js axis | 対応 | 中立値 |
|---|---|---|
| 0 / 1 | 左スティック 左右 / 上下 | 0 |
| 2 / 5 | 右スティック 左右 / 上下 | 0 |
| 3 / 4 | L2 / R2 (アナログ) | **-32767** |
| 6 / 7 | 十字キー 左右 / 上下 | 0 |

- **上方向が負** (`axis 1 = -29000` が上)。前進を正にするなら符号を反転する
- **トリガの中立は -32767**。0 を中立と誤解すると「常に半分踏まれている」扱いになる
- 軸番号が飛ばずに並ぶのは `joydev` が `ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ`
  の順に番号を振るため。`3=L2 / 4=R2 / 6,7=十字キー` を実測で固定できたので、
  残りは `0,1 = ABS_X,Y (左)` `2,5 = ABS_Z,RZ (右)` に決まる

## joystick API の読み方

8 バイト固定長のイベントを読むだけ。

```c
struct js_event {
    __u32 time;   /* ms */
    __s16 value;
    __u8  type;   /* 0x01=BUTTON, 0x02=AXIS, 0x80=INIT との OR */
    __u8  number;
};
```

Python では `struct.unpack("<IhBB", data)`。`type` に `0x80` が立っているものは
オープン直後に現在状態を通知する合成イベントなので、**操作の検出では無視する**
(本モニタは初期表示のために取り込んでいる)。
