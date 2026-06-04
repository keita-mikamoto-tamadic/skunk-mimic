#include "socket_can_comm.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h> // socket(), bind()
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

SocketCanComm::SocketCanComm() = default;

SocketCanComm::~SocketCanComm() {
  Close();
}

bool SocketCanComm::Open(const std::string& ifname) {
  if (IsOpen()) Close();  
  
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) return false;

  // カーネルインデックス取得
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    Close();
    return false;
  }
  
  // sockaddr_can アドレス更新
  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // FD enable
  int enable = 1;
  ::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
             &enable, sizeof(enable));
  
  // bind
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    Close();
    return false;
  }

  ifname_ = ifname;
  return true;
}

void SocketCanComm::Close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  } 
  // 文字列をクリア
  ifname_.clear();
}

bool SocketCanComm::IsOpen() const {
  return socket_fd_ >= 0;
}

bool SocketCanComm::SendFrame(uint32_t can_id, const uint8_t* data, size_t len, bool extended) {
  if (!IsOpen()) return false;

  struct canfd_frame frame{};
  // extended: 29bit 拡張ID(EFFフラグ付与) / それ以外: 11bit 標準ID
  frame.can_id = extended ? (can_id | CAN_EFF_FLAG) : (can_id & CAN_SFF_MASK);
  frame.len = len;
  frame.flags = CANFD_BRS;
  std::memcpy(frame.data, data, len);
  
  ssize_t written = ::write(socket_fd_, &frame, sizeof(frame));
  return written == sizeof(frame);
}

bool SocketCanComm::ReceiveFrame(int device_id, uint8_t* data, size_t* len, int timeout_ms){
  if (!IsOpen()) return false;

  uint32_t expected = (static_cast<uint32_t>(device_id) << 8) | CAN_EFF_FLAG;
  auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

  while (true) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return false;
    int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now).count());

    struct pollfd pfd = {socket_fd_, POLLIN, 0};
    if (::poll(&pfd, 1, remaining_ms) <= 0) return false;

    struct canfd_frame frame;
    ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
    if (n <= 0) return false;

    if ((frame.can_id & ~CAN_ERR_FLAG) != expected) continue;

    *len = frame.len;
    std::memcpy(data, frame.data, frame.len);
    return true;
  }
}

bool SocketCanComm::ReceiveAnyFrame(
    uint32_t* can_id_out,
    uint8_t* data,
    size_t* len,
    int timeout_ms)
{
  if (!IsOpen()) return false;

  auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

  while (true) {
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) return false;
    int remaining_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now).count());

    struct pollfd pfd = {socket_fd_, POLLIN, 0};
    if (::poll(&pfd, 1, remaining_ms) <= 0) return false;

    struct canfd_frame frame;
    ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
    if (n <= 0) return false;

    // 生の arbitration id をそのまま返す(フラグ除去のみ)。
    // device_id 抽出・フィルタは呼び側ドライバの責務。
    *can_id_out = frame.can_id & CAN_EFF_MASK;
    *len = frame.len;
    std::memcpy(data, frame.data, frame.len);
    return true;
  }
}