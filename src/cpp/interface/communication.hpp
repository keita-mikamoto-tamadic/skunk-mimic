// src/cpp/interface/communication.hpp
#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <set>

class Communication {
public:
    virtual ~Communication() = default;

    Communication(const Communication&) = delete;
    Communication& operator=(const Communication&) = delete;

    // -- Lifecycle --
    // device: トランスポート固有の識別子
    //   SocketCAN: "can0"  /  fdcanusb: "/dev/ttyACM0"
    virtual bool Open(const std::string& device) = 0;
    virtual void Close() = 0;
    virtual bool IsOpen() const = 0;

    // -- CAN Frame I/O --
    virtual bool SendFrame(uint32_t arb_id,
                           const uint8_t* data, size_t len) = 0;
    virtual bool ReceiveFrame(int device_id,
                              uint8_t* data, size_t* len,
                              int timeout_ms) = 0;

    // -- Multiple device receive --
    virtual bool ReceiveAnyFrame(
        const std::set<int>& expected_device_ids,
        int* device_id_out,
        uint8_t* data,
        size_t* len,
        int timeout_ms) = 0;

protected:
    Communication() = default;
};
