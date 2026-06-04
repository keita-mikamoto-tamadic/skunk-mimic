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
    // can_id: 送信する生 arbitration id。extended=true で 29bit 拡張フレーム、
    //         false で 11bit 標準フレーム。device/mode 等のビット配置は呼び側の責務
    //         (通信層はプロトコルの ID レイアウトを知らない)。
    virtual bool SendFrame(uint32_t can_id,
                           const uint8_t* data, size_t len,
                           bool extended) = 0;
    virtual bool ReceiveFrame(int device_id,
                              uint8_t* data, size_t* len,
                              int timeout_ms) = 0;

    // -- Multiple device receive --
    // 次に届いたフレームの生 arbitration id を can_id_out に返す。
    // device_id の抽出・フィルタは呼び側(各ドライバ)がプロトコルに従って行う。
    virtual bool ReceiveAnyFrame(
        uint32_t* can_id_out,
        uint8_t* data,
        size_t* len,
        int timeout_ms) = 0;

protected:
    Communication() = default;
};
