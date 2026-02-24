#include "state_machine.hpp"

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
}

void StateMachine::HandleStateCommand(StateCommand cmd) {
    switch (cmd) {
        case StateCommand::SERVO_ON:
            if (state_ == State::OFF) state_ = State::STOP;
            break;
        case StateCommand::SERVO_OFF:
            state_ = State::OFF;
            break;
        case StateCommand::STOP:
            if (state_ != State::OFF) state_ = State::STOP;
            break;
        case StateCommand::READY:
            if (state_ == State::STOP) {
                // 補間開始: 現在位置を記録、進捗リセット
                interp_progress_ = 0.0;
                for (size_t i = 0; i < axes_status_.size(); i++) {
                    start_positions_[i] = axes_status_[i].position;
                }
                state_ = State::READY;
            }
            break;
        case StateCommand::RUN:
            if (state_ == State::READY) state_ = State::RUN;
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
                break;
            }
        }
    }
}

State StateMachine::GetState() const { return state_; }
size_t StateMachine::GetAxisCount() const { return config_.axes.size(); }

ControlResult StateMachine::RobotController() {
    ControlResult result;
    const auto& axes = config_.axes;
    result.commands.resize(axes.size());

    for (size_t i = 0; i < axes.size(); i++) {
        AxisRef& ref = result.commands[i];
        // config から limits を埋める（全状態共通）
        ref.velocity_limit = axes[i].velocity_limit;
        ref.accel_limit = axes[i].accel_limit;
        ref.torque_limit = axes[i].torque_limit;
        ref.kp_scale = 1.0;
        ref.kv_scale = 1.0;

        switch (state_) {
            case State::OFF:
                if (position_reset_pending_) {
                    ref.motor_state = static_cast<uint8_t>(MotorState::SET_POSITION);
                    ref.ref_val = axes[i].reset_position;
                } else {
                    ref.motor_state = static_cast<uint8_t>(MotorState::OFF);
                    ref.ref_val = 0.0;
                }
                break;
            case State::STOP:
                ref.motor_state = static_cast<uint8_t>(MotorState::STOP);
                ref.ref_val = axes_status_[i].position;
                break;
            case State::READY: {
                ref.motor_state = static_cast<uint8_t>(MotorState::POSITION);
                double target;
                if (interp_progress_ < 1.0) {
                    // lib の補間関数を呼ぶ（将来）。今は線形補間
                    target = start_positions_[i]
                        + (axes[i].initial_position - start_positions_[i])
                        * interp_progress_;
                } else {
                    target = axes[i].initial_position;
                }
                ref.ref_val = target;
                break;
            }
            case State::RUN:
                // lib の制御関数を呼ぶ（将来）。今は initial_position 保持
                ref.motor_state = static_cast<uint8_t>(MotorState::POSITION);
                ref.ref_val = axes[i].initial_position;
                break;
        }
    }

    // SET_POSITION は 1tick で完了
    if (position_reset_pending_) {
        position_reset_pending_ = false;
    }

    // READY 中の補間進捗更新
    if (state_ == State::READY && interp_progress_ < 1.0) {
        interp_progress_ += tick_sec_ / config_.interpolation_time;
        if (interp_progress_ > 1.0) interp_progress_ = 1.0;
    }

    result.ready_complete = (interp_progress_ >= 1.0);
    return result;
}