/**
 * can_backend: CAN バス通信 backend
 *
 * 責務:
 *   - raw_commands (AxisRef[]) を受信してバッファ
 *   - tick 駆動で CAN 送受信 → raw_status (AxisAct[]) を出力
 *   - SCHED_FIFO / CPU affinity 設定（リアルタイム）
 *
 * 既存 device_control_manager の CAN ロジックをそのまま移植。
 */
#include <iostream>
#include <memory>
#include <vector>
#include <set>
#include <chrono>
#include "dora-node-api.h"
#include <pthread.h>
#include <sched.h>

#include "../../lib/robot_config.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/dora_helpers.hpp"
#include "../../interface/communication.hpp"
#include "../../driver/socket_can_comm.hpp"
#include "../../driver/dummy_comm.hpp"
#include "../../driver/moteus_converter.hpp"

constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputTick        = "tick";
constexpr const char* kInputRawCommands = "raw_commands";
constexpr const char* kOutputRawStatus  = "raw_status";

constexpr size_t kMaxFrameSize = 64;

static void SetCpuAffinity(uint32_t core, int32_t priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Warning: Failed to set CPU affinity" << std::endl;
    } else {
        std::cout << "[can_backend] CPU affinity set to core: " << core << std::endl;
    }

    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(current_thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Failed to set RT Process priority" << std::endl;
    } else {
        std::cout << "[can_backend] RT priority set to " << priority << std::endl;
    }
}

int main() {
    SetCpuAffinity(1, 80);
    auto node = init_dora_node();
    std::cout << "[can_backend] started" << std::endl;

    auto config = robot_config::LoadFromFile(kConfigPath);
    std::cout << "[can_backend] " << config.robot_name
              << " (" << config.axis_count << " axes)" << std::endl;

    // 通信初期化
    std::unique_ptr<Communication> can;
    if (config.transport == "dummy") {
        can = std::make_unique<DummyComm>();
    } else {
        can = std::make_unique<SocketCanComm>();
    }
    std::string device = (config.transport == "dummy") ? "dummy" : "can0";
    if (!can->Open(device)) {
        std::cerr << "[can_backend] failed to open " << device << std::endl;
        return 1;
    }
    std::cout << "[can_backend] " << config.transport << " opened" << std::endl;

    MoteusConverter converter;
    const size_t axis_count = config.axes.size();

    // コマンドバッファ
    std::vector<AxisRef> latest_commands(axis_count);
    bool has_new_commands = false;

    // expected_ids を事前構築
    std::set<int> expected_ids;
    for (const auto& ax : config.axes) {
        expected_ids.insert(ax.device_id);
    }

    while (true) {
        auto event = node.events->next();
        auto type = event_type(event);

        if (type == DoraEventType::Stop ||
            type == DoraEventType::AllInputsClosed) {
            // 全軸 OFF 送信
            uint8_t buf[kMaxFrameSize];
            AxisRef off_ref = {};
            off_ref.motor_state = MotorState::OFF;
            for (size_t i = 0; i < axis_count; i++) {
                const auto& ax = config.axes[i];
                size_t len = converter.BuildCommandFrame(
                    buf, off_ref, ax.motdir);
                can->SendFrame(
                    converter.GetArbId(ax.device_id), buf, len);
            }
            std::cout << "[can_backend] stopping (all axes OFF)" << std::endl;
            break;
        }

        if (type == DoraEventType::Input) {
            struct ArrowArray c_array;
            struct ArrowSchema c_schema;
            auto info = event_as_arrow_input_with_info(
                std::move(event),
                reinterpret_cast<uint8_t*>(&c_array),
                reinterpret_cast<uint8_t*>(&c_schema));
            std::string id(info.id);

            auto import_result = arrow::ImportArray(&c_array, &c_schema);
            if (!import_result.ok()) continue;
            auto arr = std::static_pointer_cast<arrow::UInt8Array>(
                import_result.ValueOrDie());

            if (id == kInputRawCommands) {
                latest_commands = ReceiveStructArray<AxisRef>(arr, axis_count);
                has_new_commands = true;
            }
            else if (id == kInputTick) {
                uint8_t buf[kMaxFrameSize];
                uint8_t rx[kMaxFrameSize];
                size_t rxlen;

                // 送信
                for (size_t i = 0; i < axis_count; i++) {
                    const auto& ax = config.axes[i];
                    size_t len;
                    if (has_new_commands) {
                        len = converter.BuildCommandFrame(
                            buf, latest_commands[i], ax.motdir);
                    } else {
                        len = converter.BuildQueryFrame(buf);
                    }
                    can->SendFrame(
                        converter.GetArbId(ax.device_id), buf, len);
                }
                has_new_commands = false;

                // 受信
                std::vector<AxisAct> acts(axis_count);
                std::set<int> received_ids;
                auto deadline = std::chrono::steady_clock::now()
                                + std::chrono::milliseconds(2);

                while (received_ids.size() < expected_ids.size()) {
                    auto now = std::chrono::steady_clock::now();
                    if (now >= deadline) break;

                    int remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - now).count();
                    if (remaining_ms <= 0) break;

                    int device_id;
                    if (can->ReceiveAnyFrame(expected_ids, &device_id, rx, &rxlen, remaining_ms)) {
                        for (size_t i = 0; i < axis_count; i++) {
                            if (config.axes[i].device_id == device_id) {
                                converter.ParseResponse(
                                    rx, rxlen, acts[i], config.axes[i].motdir);
                                received_ids.insert(device_id);
                                break;
                            }
                        }
                    }
                }

                SendStructArray(node, kOutputRawStatus, acts);
            }
        }
    }

    return 0;
}
