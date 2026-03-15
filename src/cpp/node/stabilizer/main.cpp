// src/cpp/node/stabilizer/main.cpp
// バランス制御ノード: IMU + モーター状態から run_command を生成
// state_status が RUN の時のみ PID 計算＋送信
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "dora-node-api.h"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/enum_def.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/pid.hpp"
#include "../../lib/dora_helpers.hpp"

// config ファイルパス（dora 実行ディレクトリ = プロジェクトルート基準）
constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputMotorStatus  = "motor_status";
constexpr const char* kInputImuData      = "imu_data";
constexpr const char* kInputStateStatus  = "state_status";
constexpr const char* kOutputRunCommand  = "run_command";

// PID 定数（旧 RCM から移動）
constexpr double kTargetPitch = 0.0465;
constexpr double kAngleKp = 12.0;
constexpr double kAngleKi = 325.0;
constexpr double kAngleKd = 0.17;
constexpr double kAngleMaxIntegral = 0.21;
constexpr double kAngleDDeadZone = 0.1;

constexpr double kVelKp = 0.0001;
constexpr double kVelKi = 0.00005;
constexpr double kVelMaxIntegral = 0.5;

constexpr double kMaxAngleOffset = 0.05;
constexpr double kMaxWheelSpeed = 30.0;

constexpr double kTickSec = 0.003;

int main() {
    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    auto config = robot_config::LoadFromFile(kConfigPath);
    std::cout << "loaded config: "
              << config.robot_name << " ("
              << config.axis_count << " axes)" << std::endl;

    const size_t axis_count = config.axes.size();

    // ホイール軸インデックス（config の name から判定）
    size_t wheel_r = SIZE_MAX;
    size_t wheel_l = SIZE_MAX;
    for (size_t i = 0; i < axis_count; i++) {
        if (config.axes[i].name == "wheel_r") wheel_r = i;
        if (config.axes[i].name == "wheel_l") wheel_l = i;
    }
    if (wheel_r == SIZE_MAX || wheel_l == SIZE_MAX) {
        std::cerr << "wheel axes not found in config" << std::endl;
        return 1;
    }
    std::cout << "wheel_r=" << wheel_r << " wheel_l=" << wheel_l << std::endl;

    // PID コントローラ
    Pid angle_pid(kAngleKp, kAngleKi, kAngleKd, kAngleMaxIntegral, kAngleDDeadZone);
    Pid velocity_pid(kVelKp, kVelKi, 0.0, kVelMaxIntegral);

    // 状態
    State state = State::OFF;
    State prev_state = State::OFF;
    std::vector<AxisAct> motor_status(axis_count);
    double pitch = 0.0;
    double pitch_rate = 0.0;

    // run_command 出力バッファ
    std::vector<AxisRef> run_command(axis_count);
    for (size_t i = 0; i < axis_count; i++) {
        run_command[i].motor_state = MotorState::OFF;
        run_command[i].ref_val = 0.0;
        run_command[i].kp_scale = 1.0;
        run_command[i].kv_scale = 1.0;
        run_command[i].velocity_limit = config.axes[i].velocity_limit;
        run_command[i].accel_limit = config.axes[i].accel_limit;
        run_command[i].torque_limit = config.axes[i].torque_limit;
    }

    while (true) {
        auto event = node.events->next();
        auto type = event_type(event);

        if (type == DoraEventType::Stop ||
            type == DoraEventType::AllInputsClosed) {
            std::cout << "stopping" << std::endl;
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

            if (id == kInputStateStatus) {
                state = ReceiveValue<State>(arr);
                // RUN に遷移したら PID リセット
                if (state == State::RUN && prev_state != State::RUN) {
                    angle_pid.Reset();
                    velocity_pid.Reset();
                    std::cout << "RUN: PID reset" << std::endl;
                }
                prev_state = state;
            }
            else if (id == kInputMotorStatus) {
                // motor_status 駆動: ステータス更新 → PID 計算 → run_command 出力
                motor_status = ReceiveStructArray<AxisAct>(arr, axis_count);

                if (state != State::RUN) continue;

                // 外側ループ: 速度PI（倒立点自動調整）
                double wheel_velocity =
                    (motor_status[wheel_r].velocity + motor_status[wheel_l].velocity) / 2.0;
                double velocity_error = 0.0 - wheel_velocity;
                double angle_offset = velocity_pid.Compute(velocity_error, 0.0, kTickSec);
                angle_offset = std::clamp(angle_offset, -kMaxAngleOffset, kMaxAngleOffset);

                // 内側ループ: 角度PID
                double effective_target = kTargetPitch + angle_offset;
                double angle_error = pitch - effective_target;
                double wheel_vel = angle_pid.Compute(angle_error, -pitch_rate, kTickSec);
                wheel_vel = std::clamp(wheel_vel, -kMaxWheelSpeed, kMaxWheelSpeed);

                // 各軸に配分
                for (size_t i = 0; i < axis_count; i++) {
                    if (i == wheel_r || i == wheel_l) {
                        run_command[i].motor_state = MotorState::VELOCITY;
                        run_command[i].ref_val = wheel_vel;
                        run_command[i].kp_scale = 0.0;
                        run_command[i].kv_scale = 20.0;
                    } else {
                        run_command[i].motor_state = MotorState::POSITION;
                        run_command[i].ref_val = config.axes[i].initial_position;
                        run_command[i].kp_scale = 1.0;
                        run_command[i].kv_scale = 1.0;
                    }
                }

                SendStructArray(node, kOutputRunCommand, run_command);
            }
            else if (id == kInputImuData) {
                auto imu = ReceiveStructArray<ImuData>(arr, 1);
                pitch = imu[0].pitch;
                pitch_rate = imu[0].gy;
            }
        }
    }

    std::cout << "finished" << std::endl;
    return 0;
}
