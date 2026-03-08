# SocketCAN セットアップと使い方

## CAN バス初期化

```bash
bash can_setup.bash
```

スクリプトの内容:
```bash
# カーネルモジュールリロード（エラーカウンターリセット）
sudo modprobe -r mttcan && sudo modprobe mttcan

# CAN FD 設定
sudo ip link set can0 down
sudo ip link set can0 type can \
  bitrate 1000000 dbitrate 5000000 \
  sample-point 0.65 dsample-point 0.8 \
  restart-ms 100 fd on
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

## 注意点

### dsample-point は 0.8 必須
mttcan (50MHz) + moteus 5Mbps の組み合わせでは `dsample-point 0.7` だと BUS-OFF になる。**必ず 0.8** を使う。

参考: https://mjbots.github.io/moteus/platforms/socketcan/

### ループバックに注意
Linux SocketCAN はデフォルトで **ループバックが有効**（`CAN_RAW_LOOPBACK=1`）。
送信したフレームが他のソケット（candump 等）に届く。

現在の `SocketCanComm` は `CAN_RAW_RECV_OWN_MSGS` がデフォルト OFF なので、
自分の送信フレームを自分で受信することはないが、
candump 等を同時に使うとループバックフレームが表示される点に留意。

もしノイズが問題になる場合は Open() 内で無効化できる:
```cpp
int loopback = 0;
setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
```

### mttcan モジュールリロード
BUS-OFF 等でエラーカウンターが溜まると通信不能になる。
`sudo modprobe -r mttcan && sudo modprobe mttcan` でリセットする。

### txqueuelen
6軸を連続送信するので `txqueuelen 1000` を設定。
デフォルト値（10）だと送信キューが溢れる可能性がある。

## 通信アーキテクチャ

### CAN ID フォーマット（moteus プロトコル）
```
送信（コマンド）: 0x8000 | device_id    Extended CAN ID + CANFD_BRS
受信（レスポンス）: device_id << 8       Extended CAN ID
```

device_id は `robot_config/mimic_v2.json` で定義:
| 軸 | name | device_id |
|----|------|-----------|
| 0 | hip_pitch_r | 50 |
| 1 | knee_r | 60 |
| 2 | wheel_r | 70 |
| 3 | hip_pitch_l | 80 |
| 4 | knee_l | 90 |
| 5 | wheel_l | 100 |

### 送受信パターン（device_control_manager ノード）
```
1. 全6軸のコマンドを連続送信（ノンブロッキング write）
2. ReceiveAnyFrame() で 10ms デッドライン内にレスポンスを収集
3. device_id で軸を特定し、ParseResponse() でデコード
4. タイムアウト時は受信済みデータのみで motor_status を送信
```

## transport の切り替え

`robot_config/mimic_v2.json` の `transport` フィールドで切り替え:

| 値 | 動作 | 用途 |
|----|------|------|
| `"dummy"` | DummyComm（CAN 不要、シミュレーション応答） | 開発・テスト |
| `"socketcan"` | SocketCanComm（can0 経由で実機通信） | 実機運用 |

## デバッグ

### CAN バス状態確認
```bash
ip -details link show can0
```

### CAN フレーム監視
```bash
# 全フレーム表示
candump can0

# 特定 device_id のみ（例: device 50 のレスポンス = 0x3200）
candump can0,3200:1FFF
```

### よくあるエラー
| 症状 | 原因 | 対処 |
|------|------|------|
| `failed to open can0` | can0 が UP していない | `bash can_setup.bash` |
| BUS-OFF | dsample-point が不正、またはハードウェア問題 | dsample-point 0.8 確認、`modprobe -r mttcan && modprobe mttcan` |
| 一部の軸だけ応答なし | CAN 配線、device_id 不一致、moteus 電源 | `candump can0` で確認 |
| 全軸タイムアウト | CAN ケーブル未接続、終端抵抗なし | 物理接続確認 |

## Linux SocketCAN API リファレンス

### ヘッダ
```cpp
#include <linux/can.h>        // can_frame, canfd_frame, CAN_EFF_FLAG, CANFD_BRS
#include <linux/can/raw.h>    // CAN_RAW, CAN_RAW_FD_FRAMES, CAN_RAW_LOOPBACK, CAN_RAW_RECV_OWN_MSGS
#include <net/if.h>           // ifreq, IFNAMSIZ
#include <sys/ioctl.h>        // ioctl, SIOCGIFINDEX
#include <sys/socket.h>       // socket, bind, setsockopt
#include <poll.h>             // poll, pollfd, POLLIN
#include <unistd.h>           // read, write, close
```

### 1. ソケット作成
```cpp
int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
// PF_CAN:  CAN プロトコルファミリー
// SOCK_RAW: RAW ソケット（フレーム単位の I/O）
// CAN_RAW:  CAN RAW プロトコル
```

### 2. インターフェースのバインド
```cpp
// インターフェース名 → カーネルインデックス
struct ifreq ifr;
strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
ioctl(fd, SIOCGIFINDEX, &ifr);

// バインド
struct sockaddr_can addr{};
addr.can_family = AF_CAN;
addr.can_ifindex = ifr.ifr_ifindex;
bind(fd, (struct sockaddr*)&addr, sizeof(addr));
```

### 3. ソケットオプション

```cpp
// CAN FD フレームの有効化（必須。これがないと canfd_frame の read/write ができない）
int enable = 1;
setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable));

// ループバック無効化（任意。他ソケットに自分の送信フレームを見せない）
int loopback = 0;
setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

// 自分の送信フレーム受信（デフォルト OFF。通常は OFF のままでよい）
int recv_own = 0;
setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));
```

| オプション | デフォルト | 説明 |
|-----------|-----------|------|
| `CAN_RAW_FD_FRAMES` | OFF | CAN FD フレーム（64バイト）の送受信を有効化 |
| `CAN_RAW_LOOPBACK` | ON | 送信フレームを同一インターフェースの他ソケットに配信 |
| `CAN_RAW_RECV_OWN_MSGS` | OFF | 自分が送信したフレームを自分でも受信する |

### 4. フレーム送信

```cpp
struct canfd_frame frame{};
frame.can_id  = 0x8032 | CAN_EFF_FLAG;  // Extended CAN ID (29bit)
frame.len     = data_len;                // 0〜64 バイト
frame.flags   = CANFD_BRS;              // Bit Rate Switch（データフェーズ高速化）
memcpy(frame.data, data, data_len);

write(fd, &frame, sizeof(frame));        // sizeof(canfd_frame) = 72
```

**フラグ一覧**:
| フラグ | 値 | 説明 |
|--------|-----|------|
| `CAN_EFF_FLAG` | `0x80000000` | Extended Frame Format（29bit ID） |
| `CAN_RTR_FLAG` | `0x40000000` | Remote Transmission Request |
| `CAN_ERR_FLAG` | `0x20000000` | エラーフレーム |
| `CANFD_BRS`    | `0x01`       | Bit Rate Switch（CAN FD データフェーズ高速化） |
| `CANFD_ESI`    | `0x02`       | Error State Indicator |

### 5. フレーム受信（タイムアウト付き）

```cpp
// poll で受信待ち（タイムアウト対応）
struct pollfd pfd = {fd, POLLIN, 0};
int ret = poll(&pfd, 1, timeout_ms);  // ret>0: データあり, 0: タイムアウト, <0: エラー

if (ret > 0) {
    struct canfd_frame frame;
    ssize_t n = read(fd, &frame, sizeof(frame));

    // CAN ID から device_id を抽出（moteus の場合）
    int device_id = (frame.can_id >> 8) & 0xFF;

    // データ取得
    size_t len = frame.len;
    uint8_t* data = frame.data;
}
```

### 6. can_frame vs canfd_frame

```
             can_frame (Classic CAN)     canfd_frame (CAN FD)
サイズ        sizeof = 16               sizeof = 72
最大データ     8 バイト                   64 バイト
flags        なし                       CANFD_BRS, CANFD_ESI
read/write   write(fd, &f, sizeof(f))   同左（CAN_RAW_FD_FRAMES 必須）
```

`CAN_RAW_FD_FRAMES` を有効にすると、read/write は常に `canfd_frame` サイズで動作する。
Classic CAN フレームも `canfd_frame` として受信される（`len <= 8`, `flags = 0`）。

### 7. ソケットクローズ

```cpp
close(fd);
```
