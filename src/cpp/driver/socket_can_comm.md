# socket_can_comm.cpp 実装例

ref: `ref/cpp/simple/motor_node/main.cc` L48-93

## インクルード

```cpp
#include "socket_can_comm.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
```

## コンストラクタ / デストラクタ

```cpp
SocketCanComm::SocketCanComm() = default;

SocketCanComm::~SocketCanComm() {
    Close();
}
```

## Open()

ref の `can_init()` に対応。

```cpp
bool SocketCanComm::Open(const std::string& ifname) {
    if (IsOpen()) Close();

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ);
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        Close();
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int enable = 1;
    ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
               &enable, sizeof(enable));

    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr),
             sizeof(addr)) < 0) {
        Close();
        return false;
    }

    ifname_ = ifname;
    return true;
}
```

### ref との差分
- ref はエラー時にソケットを閉じていない → ここでは `Close()` でリソースリーク防止
- `sockaddr_can` を `{}` で初期化（ref はゼロ初期化なし）
- C++ キャスト（`reinterpret_cast`）を使用

## Close()

```cpp
void SocketCanComm::Close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    ifname_.clear();
}
```

## IsOpen()

```cpp
bool SocketCanComm::IsOpen() const {
    return socket_fd_ >= 0;
}
```

## SendFrame()

ref の `can_send()` に対応。

```cpp
bool SocketCanComm::SendFrame(uint32_t arb_id, const uint8_t* data, size_t len) {
    if (!IsOpen()) return false;

    struct canfd_frame frame{};
    frame.can_id = arb_id | CAN_EFF_FLAG;
    frame.len = len;
    frame.flags = CANFD_BRS;
    std::memcpy(frame.data, data, len);

    ssize_t written = ::write(socket_fd_, &frame, sizeof(frame));
    return written == sizeof(frame);
}
```

### ref との差分
- ref は戻り値なし（void）→ ここでは write の結果を返す
- `IsOpen()` チェック追加
- `canfd_frame` を `{}` で初期化

## ReceiveFrame()

ref の `can_recv()` に対応。

```cpp
bool SocketCanComm::ReceiveFrame(int device_id, uint8_t* data,
                                  size_t* len, int timeout_ms) {
    if (!IsOpen()) return false;

    struct pollfd pfd = {socket_fd_, POLLIN, 0};
    if (::poll(&pfd, 1, timeout_ms) <= 0) return false;

    struct canfd_frame frame;
    ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
    if (n <= 0) return false;

    uint32_t expected = (static_cast<uint32_t>(device_id) << 8) | CAN_EFF_FLAG;
    if (frame.can_id != expected) return false;

    *len = frame.len;
    std::memcpy(data, frame.data, frame.len);
    return true;
}
```

### ref との差分
- `static_cast<uint32_t>` を追加（暗黙の型変換回避）
- `IsOpen()` チェック追加

### 既知の制限: 複数デバイス時のフレーム取り逃し
- 現在の実装は **1デバイス前提**（ref と同じ）
- 複数デバイスに送信した場合、期待と違う device_id のフレームが先に来ると捨ててしまう
- 将来の対策案:
  1. カーネルフィルタ（`setsockopt` で CAN ID フィルタ設定）
  2. 受信バッファ（device_id ごとのキューに振り分け）
  3. 不一致時のリトライ（捨てずに保存して再度 read）
