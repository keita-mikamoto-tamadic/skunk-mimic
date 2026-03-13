#include "robot_control_manager.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

constexpr size_t kWheelR = 2;
constexpr size_t kWheelL = 5;

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
                    if (i != kWheelR && i != kWheelL) {
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
                        // lib の補間関数を呼ぶ（将来）。今は線形補間
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
            // run_command をパススルー（安全管理付き）
            for (size_t i = 0; i < axes.size(); i++) {
                const auto& ref = run_command_[i];

                // SET_POSITION / OFF は RUN 中に許可しない
                if (ref.motor_state == MotorState::SET_POSITION ||
                    ref.motor_state == MotorState::OFF) {
                    continue;
                }

                commands_[i].motor_state = ref.motor_state;
                commands_[i].ref_val = ref.ref_val;
                commands_[i].kp_scale = ref.kp_scale;
                commands_[i].kv_scale = ref.kv_scale;

                // limits をコンフィグ上限でクランプ（NaN の場合はもう一方を使用）
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
            // 毎tick リセット（未受信時は前回値維持）
            run_command_received_ = false;
            break;
        }
    }
}
