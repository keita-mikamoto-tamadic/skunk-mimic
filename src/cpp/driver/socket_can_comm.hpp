#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <set>
#include "../interface/communication.hpp"

class SocketCanComm : public Communication {
  public:
    SocketCanComm();
    ~SocketCanComm() override;

    // --Lifecycle--
    bool Open(const std::string& device) override;
    void Close() override;
    bool IsOpen() const override;

    // -- CAN-FD Frame I/O --
    bool SendFrame(uint32_t arb_id, const uint8_t* data, size_t len) override;
    bool ReceiveFrame(int device_id, uint8_t* data, size_t* len, int timeout_ms) override;

    // -- Multiple device receive --
    bool ReceiveAnyFrame(
        const std::set<int>& expected_device_ids,
        int* device_id_out,
        uint8_t* data,
        size_t* len,
        int timeout_ms);

  private:
   int socket_fd_ = -1;
   std::string ifname_;

}; // SocketCanComm
