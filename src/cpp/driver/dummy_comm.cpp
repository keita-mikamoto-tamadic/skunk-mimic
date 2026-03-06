#include "dummy_comm.hpp"
#include <cmath>
#include <cstring>
#include <iostream>

bool DummyComm::Open(const std::string& device) {
    std::cout << "[DummyComm] Open(\"" << device << "\") — dummy mode"
              << std::endl;
    is_open_ = true;
    return true;
}

void DummyComm::Close() {
    is_open_ = false;
}

bool DummyComm::IsOpen() const {
    return is_open_;
}

bool DummyComm::SendFrame(uint32_t arb_id,
                           const uint8_t* /*data*/, size_t /*len*/) {
    pending_device_ids_.push(static_cast<int>(arb_id & 0x7F));
    return true;
}

bool DummyComm::ReceiveFrame(int device_id,
                              uint8_t* data, size_t* len,
                              int /*timeout_ms*/) {
    if (!is_open_) return false;
    if (pending_device_ids_.empty() || pending_device_ids_.front() != device_id)
        return false;

    pending_device_ids_.pop();
    dummy_pos_ = std::fmod(dummy_pos_ + 0.001, 10.0);
    dummy_vel_ = std::fmod(dummy_vel_ + 0.01, 10.0);
    dummy_torq_ = 0.0F;
    *len = BuildDummyResponse(data, dummy_pos_, dummy_vel_, dummy_torq_, 0);
    return true;
}

// moteus multiplex reply format でレスポンスを構築
// Query::Parse が解析できるバイト列を直接生成する
//
// バイト列:
//   [0x21][0x01][mode_i8]                         mode (reg 1, int8×1)
//   [0x2F][0x02][pos_f32][vel_f32][tor_f32]       pos/vel/tor (reg 2-4, float×3)
//   [0x21][0x0d][fault_i8]                         fault (reg 13, int8×1)
//   = 計 20 バイト
size_t DummyComm::BuildDummyResponse(
        uint8_t* buf,
        double position_rev,
        double velocity_revs,
        double torque_nm,
        uint8_t fault) {
    size_t pos = 0;

    // Block 1: mode (int8, register 0x000, 1個)
    buf[pos++] = 0x21;
    buf[pos++] = 0x00;  // register 0 (kMode)
    buf[pos++] = 0x00;  // Mode::kStopped = 0

    // Block 2: position, velocity, torque (float, registers 0x001-0x003, 3個)
    buf[pos++] = 0x2F;
    buf[pos++] = 0x01;  // register 1 (kPosition)
    float fval;
    fval = static_cast<float>(position_rev);
    std::memcpy(&buf[pos], &fval, 4); pos += 4;
    fval = static_cast<float>(velocity_revs);
    std::memcpy(&buf[pos], &fval, 4); pos += 4;
    fval = static_cast<float>(torque_nm);
    std::memcpy(&buf[pos], &fval, 4); pos += 4;

    // Block 3: fault (int8, register 0x00d, 1個)
    buf[pos++] = 0x21;
    buf[pos++] = 0x0d;  // register 13 (kFault)
    buf[pos++] = static_cast<uint8_t>(fault);

    return pos;  // 20 バイト
}

bool DummyComm::ReceiveAnyFrame(
    const std::set<int>& expected_device_ids,
    int* device_id_out,
    uint8_t* data,
    size_t* len,
    int /*timeout_ms*/)
{
    if (!is_open_) return false;

    // キューから期待される device_id を探して返す
    while (!pending_device_ids_.empty()) {
        int id = pending_device_ids_.front();
        pending_device_ids_.pop();
        if (expected_device_ids.count(id)) {
            *device_id_out = id;
            dummy_pos_ = std::fmod(dummy_pos_ + 0.001, 10.0);
            dummy_vel_ = std::fmod(dummy_vel_ + 0.01, 10.0);
            dummy_torq_ = 0.0F;
            *len = BuildDummyResponse(data, dummy_pos_, dummy_vel_, dummy_torq_, 0);
            return true;
        }
    }
    return false;
}
