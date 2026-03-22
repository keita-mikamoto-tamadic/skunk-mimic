#include "moteus_can_driver.hpp"
#include <chrono>

static constexpr size_t kMaxFrameSize = 64;

bool MoteusCanDriver::Open(const std::string& device) {
    return comm_.Open(device);
}

void MoteusCanDriver::Close() {
    comm_.Close();
}

void MoteusCanDriver::SendCommands(const std::vector<AxisRef>& commands,
                                   const std::vector<AxisConfig>& axes) {
    uint8_t buf[kMaxFrameSize];
    for (size_t i = 0; i < axes.size(); i++) {
        size_t len = converter_.BuildCommandFrame(buf, commands[i], axes[i].motdir);
        comm_.SendFrame(converter_.GetArbId(axes[i].device_id), buf, len);
    }
}

void MoteusCanDriver::SendQueries(const std::vector<AxisConfig>& axes) {
    uint8_t buf[kMaxFrameSize];
    size_t len = converter_.BuildQueryFrame(buf);
    for (const auto& ax : axes) {
        comm_.SendFrame(converter_.GetArbId(ax.device_id), buf, len);
    }
}

std::vector<AxisAct> MoteusCanDriver::ReceiveStatus(
        const std::vector<AxisConfig>& axes, int timeout_ms) {
    // expected_ids を遅延構築
    if (expected_ids_.empty()) {
        for (const auto& ax : axes) {
            expected_ids_.insert(ax.device_id);
        }
    }

    std::vector<AxisAct> acts(axes.size());
    std::set<int> received_ids;
    uint8_t rx[kMaxFrameSize];
    size_t rxlen;

    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_ms);

    while (received_ids.size() < expected_ids_.size()) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break;

        int remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now).count();
        if (remaining_ms <= 0) break;

        int device_id;
        if (comm_.ReceiveAnyFrame(expected_ids_, &device_id, rx, &rxlen, remaining_ms)) {
            for (size_t i = 0; i < axes.size(); i++) {
                if (axes[i].device_id == device_id) {
                    converter_.ParseResponse(rx, rxlen, acts[i], axes[i].motdir);
                    received_ids.insert(device_id);
                    break;
                }
            }
        }
    }

    return acts;
}

void MoteusCanDriver::SendAllOff(const std::vector<AxisConfig>& axes) {
    uint8_t buf[kMaxFrameSize];
    AxisRef off_ref = {};
    off_ref.motor_state = MotorState::OFF;
    for (const auto& ax : axes) {
        size_t len = converter_.BuildCommandFrame(buf, off_ref, ax.motdir);
        comm_.SendFrame(converter_.GetArbId(ax.device_id), buf, len);
    }
}
