#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include "../interface/communication.hpp"

// ハードウェアなしで device_control をテストするためのダミー通信。
// SendFrame で受けたコマンドは無視し、ReceiveFrame で
// moteus 互換のレスポンスバイト列（固定値）を返す。
class DummyComm : public Communication {
public:
    DummyComm() = default;
    ~DummyComm() override = default;

    bool Open(const std::string& device) override;
    void Close() override;
    bool IsOpen() const override;

    bool SendFrame(uint32_t arb_id,
                   const uint8_t* data, size_t len) override;
    bool ReceiveFrame(int device_id,
                      uint8_t* data, size_t* len,
                      int timeout_ms) override;

    bool ReceiveAnyFrame(
        const std::set<int>& expected_device_ids,
        int* device_id_out,
        uint8_t* data,
        size_t* len,
        int timeout_ms) override;

private:
    bool is_open_ = false;
    int last_sent_device_id_ = -1;
    double dummy_pos_ = 0.0;
    double dummy_vel_ = 0.0;
    double dummy_torq_ = 0.0;

    // moteus multiplex reply フォーマットのレスポンスバイト列を構築
    static size_t BuildDummyResponse(uint8_t* buf,
                                     double position_rev,
                                     double velocity_revs,
                                     double torque_nm,
                                     uint8_t fault);
};
