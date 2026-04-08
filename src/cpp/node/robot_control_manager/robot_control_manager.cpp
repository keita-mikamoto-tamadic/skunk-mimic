#include "robot_control_manager.hpp"
#include "../../controller/angle_pid_controller.hpp"
#include "../../controller/lqr_controller.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

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

RobotControlManager::RobotControlManager()
    : state_(State::OFF), interp_progress_(0.0), tick_sec_(0.003),
      position_reset_pending_(false), run_command_received_(false) {}

void RobotControlManager::Configure(
    const RobotConfig& config, FaultEvaluator fault_evaluator,
    double tick_sec) {
    config_ = config;
    axes_status_.resize(config.axes.size());
    fault_evaluator_ = std::move(fault_evaluator);
    tick_sec_ = tick_sec;
    start_positions_.resize(config.axes.size(), 0.0);
    torque_limit_count_.resize(config.axes.size(), 0);

    // commands_ を初期化（limits は静的なのでここで1回だけ設定）
    commands_.resize(config.axes.size());
    for (size_t i = 0; i < config.axes.size(); i++) {
        commands_[i].velocity_limit = config.axes[i].velocity_limit;
        commands_[i].accel_limit = config.axes[i].accel_limit;
        commands_[i].torque_limit = config.axes[i].torque_limit;
        commands_[i].kp_scale = 1.0;
        commands_[i].kv_scale = 1.0;
        commands_[i].motor_state = MotorState::OFF;
        commands_[i].ref_val = 0.0;
    }

    run_command_.resize(config.axes.size());

    // Controller + EKF 初期化
    controller_ = CreateController(config.controller, config);
    std::cout << "controller: " << config.controller << std::endl;
    ekf_.Init();

    // ホイール軸インデックス解決
    for (size_t i = 0; i < config.axes.size(); i++) {
        if (config.axes[i].name == "wheel_r") wheel_r_ = i;
        if (config.axes[i].name == "wheel_l") wheel_l_ = i;
    }
}

void RobotControlManager::HandleStateCommand(StateCommand cmd) {
    switch (cmd) {
        case StateCommand::SERVO_ON:
            if (state_ == State::OFF) {
                state_ = State::STOP;
                for (size_t i = 0; i < commands_.size(); i++) {
                    commands_[i].motor_state = MotorState::STOP;
                    commands_[i].kp_scale = 1.0;
                    commands_[i].kv_scale = 1.0;
                }
            }
            break;
        case StateCommand::SERVO_OFF:
            state_ = State::OFF;
            for (size_t i = 0; i < commands_.size(); i++) {
                commands_[i].motor_state = MotorState::OFF;
                commands_[i].ref_val = 0.0;
                commands_[i].kp_scale = 1.0;
                commands_[i].kv_scale = 1.0;
            }
            break;
        case StateCommand::STOP:
            if (state_ != State::OFF) {
                state_ = State::STOP;
                for (size_t i = 0; i < commands_.size(); i++) {
                    commands_[i].motor_state = MotorState::STOP;
                    commands_[i].kp_scale = 1.0;
                    commands_[i].kv_scale = 1.0;
                }
            }
            break;
        case StateCommand::READY:
            if (state_ == State::STOP) {
                interp_progress_ = 0.0;
                for (size_t i = 0; i < axes_status_.size(); i++) {
                    start_positions_[i] = axes_status_[i].position;
                }
                state_ = State::READY;
                for (size_t i = 0; i < commands_.size(); i++) {
                    if (config_.axes[i].name != "wheel_r" &&
                        config_.axes[i].name != "wheel_l") {
                        commands_[i].motor_state = MotorState::POSITION;
                        commands_[i].kp_scale = 1.0;
                        commands_[i].kv_scale = 1.0;
                    } else {
                        commands_[i].motor_state = MotorState::VELOCITY;
                        commands_[i].kp_scale = 0.0;
                        commands_[i].kv_scale = 20.0;
                    }
                }
            }
            break;
        case StateCommand::RUN:
            if (state_ == State::READY) {
                state_ = State::RUN;
                controller_->Reset();
                std::cout << "RUN: controller reset" << std::endl;
            }
            break;
        case StateCommand::INIT_POSITION_RESET:
            if (state_ == State::OFF) {
                position_reset_pending_ = true;
            }
            break;
    }
}

void RobotControlManager::UpdateMotorStatus(const std::vector<AxisAct>& status) {
    axes_status_ = status;

    // フォルト検出: evaluator が注入されていれば使用
    if (fault_evaluator_) {
        for (const auto& s : axes_status_) {
            auto result = fault_evaluator_(s.fault);
            if (result.has_value()) {
                state_ = result.value();
                for (size_t i = 0; i < commands_.size(); i++) {
                    commands_[i].motor_state = MotorState::OFF;
                    commands_[i].ref_val = 0.0;
                }
                break;
            }
        }
    }

    // トルクリミット連続ヒット監視（軸ごと）
    constexpr int kTorqueLimitThreshold = 100;
    for (size_t i = 0; i < axes_status_.size(); i++) {
        if (std::abs(axes_status_[i].torque) >= config_.axes[i].torque_limit) {
            torque_limit_count_[i]++;
            if (torque_limit_count_[i] >= kTorqueLimitThreshold) {
                commands_[i].motor_state = MotorState::OFF;
                commands_[i].ref_val = 0.0;
                std::cerr << "axis " << i
                          << " servo OFF: torque limit hit "
                          << kTorqueLimitThreshold << " consecutive ticks"
                          << std::endl;
            }
        } else {
            torque_limit_count_[i] = 0;
        }
    }

    // EKF 更新（READY 以降）
    if (state_ >= State::READY && wheel_r_ != SIZE_MAX) {
        double pitch = imu_data_.pitch;
        double pitch_rate = imu_data_.gy;
        double ax_forward = imu_data_.ax * std::cos(pitch)
                          + imu_data_.az * std::sin(pitch);
        ekf_.Predict(static_cast<float>(tick_sec_),
                     static_cast<float>(ax_forward));

        double wheel_vel_avg =
            (axes_status_[wheel_r_].velocity + axes_status_[wheel_l_].velocity) / 2.0;
        double v_body = kWheelRadius * wheel_vel_avg
                      + kCoMHeight * std::cos(pitch) * pitch_rate;
        ekf_.Correct(static_cast<float>(v_body));
    }
}

void RobotControlManager::UpdateImuData(const ImuData& imu) {
    imu_data_ = imu;
}

void RobotControlManager::UpdateRunCommand(const std::vector<AxisRef>& run_command) {
    run_command_ = run_command;
    run_command_received_ = true;
}

State RobotControlManager::GetState() const { return state_; }
size_t RobotControlManager::GetAxisCount() const { return config_.axes.size(); }

const std::vector<AxisRef>& RobotControlManager::GetCommands() const {
    return commands_;
}

bool RobotControlManager::IsReadyComplete() const {
    return interp_progress_ >= 1.0;
}

EstimatedState RobotControlManager::GetEstimatedState() const {
    return {static_cast<double>(ekf_.est_velocity()), 0.0, 0.0};
}

void RobotControlManager::RobotController() {
    const auto& axes = config_.axes;

    switch (state_) {
        case State::OFF:
            if (position_reset_pending_) {
                for (size_t i = 0; i < axes.size(); i++) {
                    commands_[i].motor_state = MotorState::SET_POSITION;
                    commands_[i].ref_val = axes[i].reset_position;
                }
                position_reset_pending_ = false;
            }
            break;

        case State::STOP:
            for (size_t i = 0; i < axes.size(); i++) {
                commands_[i].ref_val = axes_status_[i].position;
            }
            break;

        case State::READY:
            for (size_t i = 0; i < axes.size(); i++) {
                if (commands_[i].motor_state == MotorState::VELOCITY) {
                    commands_[i].ref_val = 0.0;
                } else {
                    double target;
                    if (interp_progress_ < 1.0) {
                        target = start_positions_[i]
                            + (axes[i].initial_position - start_positions_[i])
                            * interp_progress_;
                    } else {
                        target = axes[i].initial_position;
                    }
                    commands_[i].ref_val = target;
                }
            }
            // 補間進捗更新
            if (interp_progress_ < 1.0) {
                interp_progress_ += tick_sec_ / config_.interpolation_time;
                if (interp_progress_ > 1.0) interp_progress_ = 1.0;
            }
            break;

        case State::RUN: {
            // Controller で制御計算
            controller_->Update(axes_status_, imu_data_, ekf_);
            auto ctrl_commands = controller_->Compute(config_);

            // 制御出力を commands_ に反映（安全管理付き）
            for (size_t i = 0; i < axes.size(); i++) {
                const auto& ref = ctrl_commands[i];

                if (ref.motor_state == MotorState::SET_POSITION ||
                    ref.motor_state == MotorState::OFF) {
                    continue;
                }

                commands_[i].motor_state = ref.motor_state;
                commands_[i].ref_val = ref.ref_val;
                commands_[i].kp_scale = ref.kp_scale;
                commands_[i].kv_scale = ref.kv_scale;

                // limits をコンフィグ上限でクランプ
                commands_[i].velocity_limit = std::isnan(ref.velocity_limit)
                    ? axes[i].velocity_limit
                    : (std::isnan(axes[i].velocity_limit)
                        ? ref.velocity_limit
                        : std::min(ref.velocity_limit, axes[i].velocity_limit));
                commands_[i].accel_limit = std::isnan(ref.accel_limit)
                    ? axes[i].accel_limit
                    : (std::isnan(axes[i].accel_limit)
                        ? ref.accel_limit
                        : std::min(ref.accel_limit, axes[i].accel_limit));
                commands_[i].torque_limit = std::isnan(ref.torque_limit)
                    ? axes[i].torque_limit
                    : std::min(ref.torque_limit, axes[i].torque_limit);
            }
            break;
        }
    }
}
