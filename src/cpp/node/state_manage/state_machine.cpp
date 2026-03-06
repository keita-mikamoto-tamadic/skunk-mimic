#include "state_machine.hpp"
#include <cmath>
#include <iostream>

StateMachine::StateMachine()
    : state_(State::OFF), interp_progress_(0.0), tick_sec_(0.003),
      position_reset_pending_(false) {}

void StateMachine::Configure(
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
}

void StateMachine::HandleStateCommand(StateCommand cmd) {
    switch (cmd) {
        case StateCommand::SERVO_ON:
            if (state_ == State::OFF) {
                state_ = State::STOP;
                for (size_t i = 0; i < commands_.size(); i++) {
                    commands_[i].motor_state = MotorState::STOP;
                }
            }
            break;
        case StateCommand::SERVO_OFF:
            state_ = State::OFF;
            for (size_t i = 0; i < commands_.size(); i++) {
                commands_[i].motor_state = MotorState::OFF;
                commands_[i].ref_val = 0.0;
            }
            break;
        case StateCommand::STOP:
            if (state_ != State::OFF) {
                state_ = State::STOP;
                for (size_t i = 0; i < commands_.size(); i++) {
                    commands_[i].motor_state = MotorState::STOP;
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
                    if (i != 2 && i != 5) { 
                        commands_[i].motor_state = MotorState::POSITION;
                    } else commands_[i].motor_state = MotorState::VELOCITY;
                }
            }
            break;
        case StateCommand::RUN:
            if (state_ == State::READY) state_ = State::RUN;
            // motor_state は READY で設定した POSITION がそのまま保持される
            // 将来: 制御ロジックがここで軸ごとに motor_state を変更
            break;
        case StateCommand::INIT_POSITION_RESET:
            if (state_ == State::OFF) {
                position_reset_pending_ = true;
            }
            break;
    }
}

void StateMachine::UpdateMotorStatus(const std::vector<AxisAct>& status) {
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
                std::cerr << "[state_manager] axis " << i
                          << " servo OFF: torque limit hit "
                          << kTorqueLimitThreshold << " consecutive ticks"
                          << std::endl;
            }
        } else {
            torque_limit_count_[i] = 0;
        }
    }
}

State StateMachine::GetState() const { return state_; }
size_t StateMachine::GetAxisCount() const { return config_.axes.size(); }

const std::vector<AxisRef>& StateMachine::GetCommands() const {
    return commands_;
}

bool StateMachine::IsReadyComplete() const {
    return interp_progress_ >= 1.0;
}

void StateMachine::RobotController() {
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

        case State::RUN:
            // 将来: PID計算をここで1回行い、結果をループ内で各軸に配分
            for (size_t i = 0; i < axes.size(); i++) {
                if (commands_[i].motor_state == MotorState::POSITION) commands_[i].ref_val = axes[i].initial_position;
                if (commands_[i].motor_state == MotorState::VELOCITY) commands_[i].ref_val = 0.0F;
            }
            break;
    }
}
