/**
 * device_control_manager: CAN バス通信 + motor_status/imu_data 出力
 *
 * 責務:
 *   - tick 駆動で CAN 送受信 → motor_status 出力
 *   - motor_commands 受信 → コマンドバッファ更新
 *   - imu_data 受信 → imu_data パススルー出力
 *   - SCHED_FIFO / CPU affinity 設定（リアルタイム）
 *
 * Communication インターフェースで transport を切り替え:
 *   "socketcan" → SocketCanComm（CAN-FD）
 *   "dummy"     → DummyComm（テスト用）
 *   将来: "rs485" → Rs485Comm 等
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
constexpr const char* kInputTick          = "tick";
constexpr const char* kInputMotorCommands = "motor_commands";
constexpr const char* kInputImuData       = "raw_imu";
constexpr const char* kOutputMotorStatus  = "motor_status";
constexpr const char* kOutputImuData      = "imu_data";
constexpr const char* kOutputLatency      = "latency";

constexpr size_t kMaxFrameSize = 64;

static void SetCpuAffinity(uint32_t core, int32_t priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Warning: Failed to set CPU affinity" << std::endl;
    }

    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(current_thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Failed to set RT Process priority" << std::endl;
    }
}

int main() {
    SetCpuAffinity(1, 80);
    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    auto config = robot_config::LoadFromFile(kConfigPath);
    std::cout <<config.robot_name
              << " (" << config.axis_count << " axes)" << std::endl;

    // 通信初期化（transport に応じて実装を切り替え）
    std::unique_ptr<Communication> can;
    if (config.transport == "dummy") {
        can = std::make_unique<DummyComm>();
    } else {
        can = std::make_unique<SocketCanComm>();
    }
    std::string device = (config.transport == "dummy") ? "dummy" : "can0";
    if (!can->Open(device)) {
        std::cerr << "failed to open " << device << std::endl;
        return 1;
    }
    std::cout <<config.transport << " opened" << std::endl;

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

    // レイテンシ計測
    int can_count = 0;
    long can_sum = 0;
    long can_max = 0;
    std::chrono::steady_clock::time_point status_send_time;
    bool status_pending = false;
    int ctrl_count = 0;
    long ctrl_sum = 0;
    long ctrl_max = 0;

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
            std::cout << "stopping (all axes OFF)" << std::endl;
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

            // IMU パススルー（ゼロコピー）
            if (id == kInputImuData) {
                send_arrow_output(
                    node.send_output, rust::String(kOutputImuData),
                    reinterpret_cast<uint8_t*>(&c_array),
                    reinterpret_cast<uint8_t*>(&c_schema));
                continue;
            }

            // 以下は Arrow Import が必要
            auto import_result = arrow::ImportArray(&c_array, &c_schema);
            if (!import_result.ok()) continue;
            auto arr = std::static_pointer_cast<arrow::UInt8Array>(
                import_result.ValueOrDie());

            if (id == kInputMotorCommands) {
                // 制御側計測: motor_status 送信 → motor_commands 受信
                if (status_pending) {
                    long us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - status_send_time).count();
                    status_pending = false;
                    ctrl_sum += us;
                    if (us > ctrl_max) ctrl_max = us;
                    ctrl_count++;
                }
                latest_commands = ReceiveStructArray<AxisRef>(arr, axis_count);
                has_new_commands = true;
            }
            else if (id == kInputTick) {
                uint8_t buf[kMaxFrameSize];
                uint8_t rx[kMaxFrameSize];
                size_t rxlen;
                auto t0 = std::chrono::steady_clock::now();

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

                // CAN レイテンシ計測
                long us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - t0).count();
                can_sum += us;
                if (us > can_max) can_max = us;
                can_count++;

                // latency データ送信（毎 tick）
                LatencyData latency;
                latency.can_avg_us = (can_count > 0) ? static_cast<double>(can_sum) / can_count : 0;
                latency.can_max_us = static_cast<double>(can_max);
                latency.ctrl_avg_us = (ctrl_count > 0) ? static_cast<double>(ctrl_sum) / ctrl_count : 0;
                latency.ctrl_max_us = static_cast<double>(ctrl_max);
                SendStruct(node, kOutputLatency, latency);

                SendStructArray(node, kOutputMotorStatus, acts);
                status_send_time = std::chrono::steady_clock::now();
                status_pending = true;
            }
        }
    }

    return 0;
}
