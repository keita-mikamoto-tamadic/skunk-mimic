// src/cpp/node/stabilizer/main.cpp
// バランス制御ノード: IMU + モーター状態から run_command を生成
// state_status が RUN の時のみ制御計算＋送信
// Controller 実装は robot_config の "controller" フィールドで選択
#include <iostream>
#include <memory>
#include <vector>
#include "dora-node-api.h"
#include "../../lib/shm_data_format.hpp"
#include "../../lib/enum_def.hpp"
#include "../../lib/robot_config.hpp"
#include "../../lib/dora_helpers.hpp"
#include "controller.hpp"
#include "angle_pid_controller.hpp"
#include "lqr_controller.hpp"
#include "body_state_ekf.hpp"

constexpr const char* kConfigPath = "robot_config/mimic_v2.json";

// 入出力ID
constexpr const char* kInputMotorStatus  = "motor_status";
constexpr const char* kInputImuData      = "imu_data";
constexpr const char* kInputStateStatus  = "state_status";
constexpr const char* kOutputRunCommand  = "run_command";
constexpr const char* kOutputEstState    = "estimated_state";

// コントローラ生成
static std::unique_ptr<Controller> CreateController(
    const std::string& name, const RobotConfig& config) {
    if (name == "angle_pid") {
        return std::make_unique<AnglePidController>(config);
    }
    if (name == "lqr") {
        return std::make_unique<LqrController>(config);
    }
    std::cerr << "unknown controller: " << name
              << ", falling back to angle_pid" << std::endl;
    return std::make_unique<AnglePidController>(config);
}

int main() {
    auto node = init_dora_node();
    std::cout << "started" << std::endl;

    auto config = robot_config::LoadFromFile(kConfigPath);
    const size_t axis_count = config.axes.size();
    std::cout << config.robot_name
              << " (" << axis_count << " axes)" << std::endl;

    auto controller = CreateController(config.controller, config);
    std::cout << "controller: " << config.controller << std::endl;

    // 状態
    State state = State::OFF;
    State prev_state = State::OFF;
    ImuData imu_data = {};

    // EKF: コントローラ非依存のボディ状態推定
    BodyStateEkf ekf;
    ekf.Init();
    constexpr double kWheelRadius = 0.07795;
    constexpr double kCoMHeight = 0.2484;
    constexpr double kTickSec = 0.003;

    // ホイール軸インデックス
    size_t wheel_r = SIZE_MAX, wheel_l = SIZE_MAX;
    for (size_t i = 0; i < axis_count; i++) {
        if (config.axes[i].name == "wheel_r") wheel_r = i;
        if (config.axes[i].name == "wheel_l") wheel_l = i;
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
                if (state == State::RUN && prev_state != State::RUN) {
                    controller->Reset();
                    std::cout << "RUN: controller reset" << std::endl;
                }
                prev_state = state;
            }
            else if (id == kInputImuData) {
                auto imu_vec = ReceiveStructArray<ImuData>(arr, 1);
                imu_data = imu_vec[0];
            }
            else if (id == kInputMotorStatus) {
                auto motor_status = ReceiveStructArray<AxisAct>(arr, axis_count);

                // READY以降: EKF更新（コントローラ非依存）
                if (state >= State::READY && wheel_r != SIZE_MAX) {
                    double pitch = imu_data.pitch;
                    double pitch_rate = imu_data.gy;
                    double ax_forward = imu_data.ax * std::cos(pitch)
                                      + imu_data.az * std::sin(pitch);
                    ekf.Predict(static_cast<float>(kTickSec),
                                static_cast<float>(ax_forward));

                    double wheel_vel_avg =
                        (motor_status[wheel_r].velocity + motor_status[wheel_l].velocity) / 2.0;
                    double v_body = kWheelRadius * wheel_vel_avg
                                  + kCoMHeight * std::cos(pitch) * pitch_rate;
                    ekf.Correct(static_cast<float>(v_body));
                }

                // EstimatedState は常に送信
                EstimatedState est_state = {
                    static_cast<double>(ekf.est_velocity()), 0.0, 0.0};
                SendStruct(node, kOutputEstState, est_state);

                // RUNのみ: 制御出力
                if (state != State::RUN) continue;

                controller->Update(motor_status, imu_data, ekf);
                auto run_command = controller->Compute(config);
                ZeroCopySendStructArray(node, kOutputRunCommand, run_command);
            }
        }
    }

    std::cout << "finished" << std::endl;
    return 0;
}
