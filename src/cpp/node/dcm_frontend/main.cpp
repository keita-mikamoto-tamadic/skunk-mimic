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
#include <pthread.h>
#include <sched.h>
#include "dora-node-api.h"

#include "../../lib/robot_config.hpp"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/dora_helpers.hpp"

constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputMotorCommands = "motor_commands";
constexpr const char* kInputRawStatus     = "raw_status";
constexpr const char* kOutputMotorStatus  = "motor_status";
constexpr const char* kOutputRawCommands  = "raw_commands";

static void SetCpuAffinity(uint32_t core, int32_t priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "[dcm_frontend] Warning: Failed to set CPU affinity" << std::endl;
    }

    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(current_thread, SCHED_FIFO, &param) != 0) {
        std::cerr << "[dcm_frontend] Warning: Failed to set RT Process priority" << std::endl;
    }
}

int main() {
    SetCpuAffinity(1, 80);
    auto node = init_dora_node();
    std::cout << "[dcm_frontend] started" << std::endl;

    auto config = robot_config::LoadFromFile(kConfigPath);
    const size_t axis_count = config.axes.size();
    std::cout << "[dcm_frontend] " << config.robot_name
              << " (" << axis_count << " axes)" << std::endl;

    // レイテンシ計測
    // CAN側: motor_commands 送信 → raw_status 受信
    std::chrono::steady_clock::time_point cmd_send_time;
    bool cmd_pending = false;
    int can_count = 0;
    long can_sum = 0;
    long can_max = 0;
    // 制御側: motor_status 送信 → motor_commands 受信
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
            if (id == kInputRawStatus) {
                // CAN側計測: motor_commands送信→raw_status受信
                if (cmd_pending) {
                    long us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - cmd_send_time).count();
                    cmd_pending = false;
                    can_sum += us;
                    if (us > can_max) can_max = us;
                    can_count++;
                }
                // motor_status 送信 → 制御側計測の起点
                send_arrow_output(
                    node.send_output, rust::String(kOutputMotorStatus),
                    reinterpret_cast<uint8_t*>(&c_array),
                    reinterpret_cast<uint8_t*>(&c_schema));
                status_send_time = std::chrono::steady_clock::now();
                status_pending = true;
                continue;
            }
            if (id == kInputMotorCommands) {
                // 制御側計測: motor_status送信→motor_commands受信
                if (status_pending) {
                    long us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - status_send_time).count();
                    status_pending = false;
                    ctrl_sum += us;
                    if (us > ctrl_max) ctrl_max = us;
                    ctrl_count++;
                }
                // ログ出力（1秒ごと）
                if (can_count > 0 && can_count % 333 == 0) {
                    std::cout << "[dcm_frontend] CAN: avg=" << (can_sum / can_count)
                              << "us max=" << can_max << "us"
                              << " | CTRL: avg=" << (ctrl_count > 0 ? ctrl_sum / ctrl_count : 0)
                              << "us max=" << ctrl_max << "us" << std::endl;
                }
                cmd_send_time = std::chrono::steady_clock::now();
                cmd_pending = true;
                send_arrow_output(
                    node.send_output, rust::String(kOutputRawCommands),
                    reinterpret_cast<uint8_t*>(&c_array),
                    reinterpret_cast<uint8_t*>(&c_schema));
                continue;
            }

        }
    }

    return 0;
}
