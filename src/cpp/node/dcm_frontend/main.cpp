/**
 * dcm_frontend: motor_commands <-> raw_commands/raw_status ルーター + ウォッチドッグ
 *
 * 責務:
 *   1. motor_commands 受信 → raw_commands として backend に即転送
 *   2. raw_status 受信 → motor_status として RCM に転送 + ウォッチドッグリセット
 *   3. raw_imu 受信 → imu_data として下流に転送
 *   4. ウォッチドッグ (30ms): タイムアウト時に全軸エラー AxisAct を送信
 */
#include <iostream>
#include <vector>
#include <chrono>
#include "dora-node-api.h"

#include "../../lib/robot_config.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/dora_helpers.hpp"

constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputMotorCommands = "motor_commands";
constexpr const char* kInputRawStatus     = "raw_status";
constexpr const char* kInputRawImu        = "raw_imu";
constexpr const char* kInputWatchdog      = "watchdog";
constexpr const char* kOutputMotorStatus  = "motor_status";
constexpr const char* kOutputRawCommands  = "raw_commands";
constexpr const char* kOutputImuData      = "imu_data";

int main() {
    auto node = init_dora_node();
    std::cout << "[dcm_frontend] started" << std::endl;

    auto config = robot_config::LoadFromFile(kConfigPath);
    const size_t axis_count = config.axes.size();
    std::cout << "[dcm_frontend] " << config.robot_name
              << " (" << axis_count << " axes)" << std::endl;

    // ウォッチドッグ: raw_status の最終受信時刻
    auto last_status_time = std::chrono::steady_clock::now();
    bool watchdog_tripped = false;

    while (true) {
        auto event = node.events->next();
        auto type = event_type(event);

        if (type == DoraEventType::Stop ||
            type == DoraEventType::AllInputsClosed) {
            std::cout << "[dcm_frontend] stopping" << std::endl;
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

            // パススルー経路: Import せず C ArrowArray を直接転送（ゼロコピー）
            if (id == kInputRawImu) {
                send_arrow_output(
                    node.send_output, rust::String(kOutputImuData),
                    reinterpret_cast<uint8_t*>(&c_array),
                    reinterpret_cast<uint8_t*>(&c_schema));
                continue;
            }
            if (id == kInputRawStatus) {
                send_arrow_output(
                    node.send_output, rust::String(kOutputMotorStatus),
                    reinterpret_cast<uint8_t*>(&c_array),
                    reinterpret_cast<uint8_t*>(&c_schema));
                last_status_time = std::chrono::steady_clock::now();
                watchdog_tripped = false;
                continue;
            }
            if (id == kInputMotorCommands) {
                send_arrow_output(
                    node.send_output, rust::String(kOutputRawCommands),
                    reinterpret_cast<uint8_t*>(&c_array),
                    reinterpret_cast<uint8_t*>(&c_schema));
                continue;
            }

            // ウォッチドッグ: Import が必要
            if (id == kInputWatchdog) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_status_time).count();

                if (elapsed > 30 && !watchdog_tripped) {
                    std::vector<AxisAct> error_status(axis_count);
                    for (size_t i = 0; i < axis_count; i++) {
                        error_status[i].position = 0.0;
                        error_status[i].velocity = 0.0;
                        error_status[i].torque = 0.0;
                        error_status[i].fault = 1;
                    }
                    SendStructArray(node, kOutputMotorStatus, error_status);
                    watchdog_tripped = true;
                    std::cerr << "[dcm_frontend] watchdog timeout ("
                              << elapsed << "ms)" << std::endl;
                }
                // watchdog の c_array は使わないので release
                if (c_array.release) c_array.release(&c_array);
                if (c_schema.release) c_schema.release(&c_schema);
            }
        }
    }

    return 0;
}
