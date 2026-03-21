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

            auto import_result = arrow::ImportArray(&c_array, &c_schema);
            if (!import_result.ok()) continue;
            auto arr = std::static_pointer_cast<arrow::UInt8Array>(
                import_result.ValueOrDie());

            if (id == kInputMotorCommands) {
                // motor_commands → raw_commands: 受信即転送
                auto commands = ReceiveStructArray<AxisRef>(arr, axis_count);
                SendStructArray(node, kOutputRawCommands, commands);
            }
            else if (id == kInputRawImu) {
                // raw_imu → imu_data: パススルー
                struct ArrowArray out_array;
                struct ArrowSchema out_schema;
                if (arrow::ExportArray(*arr, &out_array, &out_schema).ok()) {
                    send_arrow_output(
                        node.send_output, rust::String(kOutputImuData),
                        reinterpret_cast<uint8_t*>(&out_array),
                        reinterpret_cast<uint8_t*>(&out_schema));
                }
            }
            else if (id == kInputRawStatus) {
                // raw_status → motor_status: 転送 + ウォッチドッグリセット
                auto status = ReceiveStructArray<AxisAct>(arr, axis_count);
                SendStructArray(node, kOutputMotorStatus, status);
                last_status_time = std::chrono::steady_clock::now();
                watchdog_tripped = false;
            }
            else if (id == kInputWatchdog) {
                // 30ms タイマー: タイムアウト判定
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_status_time).count();

                if (elapsed > 30 && !watchdog_tripped) {
                    // タイムアウト: 全軸エラー AxisAct を合成して送信
                    std::vector<AxisAct> error_status(axis_count);
                    for (size_t i = 0; i < axis_count; i++) {
                        error_status[i].position = 0.0;
                        error_status[i].velocity = 0.0;
                        error_status[i].torque = 0.0;
                        error_status[i].fault = 1;  // 異常
                    }
                    SendStructArray(node, kOutputMotorStatus, error_status);
                    watchdog_tripped = true;
                    std::cerr << "[dcm_frontend] watchdog timeout ("
                              << elapsed << "ms)" << std::endl;
                }
            }
        }
    }

    return 0;
}
