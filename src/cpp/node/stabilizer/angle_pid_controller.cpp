#include "angle_pid_controller.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

AnglePidController::AnglePidController(const RobotConfig& config)
    : angle_pid_(kAngleKp, kAngleKi, kAngleKd, kAngleMaxIntegral, kAngleDDeadZone),
      velocity_pid_(kVelKp, kVelKi, 0.0, kVelMaxIntegral)
{
    const size_t axis_count = config.axes.size();

    // ホイール軸インデックスを解決
    for (size_t i = 0; i < axis_count; i++) {
        if (config.axes[i].name == "wheel_r") wheel_r_ = i;
        if (config.axes[i].name == "wheel_l") wheel_l_ = i;
    }
    if (wheel_r_ == SIZE_MAX || wheel_l_ == SIZE_MAX) {
        std::cerr << "wheel axes not found" << std::endl;
    }
    std::cout << "wheel_r=" << wheel_r_
              << " wheel_l=" << wheel_l_ << std::endl;

    // run_command バッファ初期化
    run_command_.resize(axis_count);
    for (size_t i = 0; i < axis_count; i++) {
        run_command_[i].motor_state = MotorState::OFF;
        run_command_[i].ref_val = 0.0;
        run_command_[i].kp_scale = 1.0;
        run_command_[i].kv_scale = 1.0;
        run_command_[i].velocity_limit = config.axes[i].velocity_limit;
        run_command_[i].accel_limit = config.axes[i].accel_limit;
        run_command_[i].torque_limit = config.axes[i].torque_limit;
    }
}

void AnglePidController::Reset() {
    angle_pid_.Reset();
    velocity_pid_.Reset();
    drive_forward_ = 0.0;
    drive_yaw_ = 0.0;
}

void AnglePidController::SetDriveCommand(const DriveCommand& cmd) {
    // forward/yaw は正規化 -1..1。クランプしてからフルスケールを掛ける
    drive_forward_ = std::clamp(cmd.forward, -1.0, 1.0) * kMaxDriveWheelVel;
    drive_yaw_     = std::clamp(cmd.yaw,     -1.0, 1.0) * kMaxYawWheelDiff;
}

void AnglePidController::Update(const std::vector<AxisAct>& motor_status,
                                const ImuData& imu_data,
                                const BodyStateEkf& /* ekf */) {
    motor_status_ = motor_status;
    pitch_ = imu_data.pitch;
    pitch_rate_ = imu_data.gy;
}

std::vector<AxisRef> AnglePidController::Compute(const RobotConfig& config) {
    // 外側ループ: 速度PI（倒立点自動調整 + 走行指令の追従）
    // 走行指令はここ (目標速度) に入れる。PID 出力への直接加算は、この外側
    // ループが v=0 に引き戻すため打ち消されて効かない (持続成分は目標側に入れる)
    double wheel_velocity =
        (motor_status_[wheel_r_].velocity + motor_status_[wheel_l_].velocity) / 2.0;
    double velocity_error = drive_forward_ - wheel_velocity;
    double angle_offset = velocity_pid_.Compute(velocity_error, 0.0, kTickSec);
    angle_offset = std::clamp(angle_offset, -kMaxAngleOffset, kMaxAngleOffset);

    // 内側ループ: 角度PID
    double effective_target = kTargetPitch + angle_offset;
    double angle_error = pitch_ - effective_target;
    double wheel_vel = angle_pid_.Compute(angle_error, -pitch_rate_, kTickSec);
    wheel_vel = std::clamp(wheel_vel, -kMaxWheelSpeed, kMaxWheelSpeed);

    // 各軸に配分。旋回は左右差動 (共通モード = バランス+前後、差動 = 旋回で直交)。
    // 符号: 右に倒す (drive_yaw_ 正) と右輪減速・左輪増速 = 右旋回のつもり。
    // 実機で逆だったら drive_yaw_ の符号をここで反転する
    const size_t axis_count = config.axes.size();
    for (size_t i = 0; i < axis_count; i++) {
        if (i == wheel_r_ || i == wheel_l_) {
            double diff = (i == wheel_r_) ? -drive_yaw_ : +drive_yaw_;
            run_command_[i].motor_state = MotorState::VELOCITY;
            run_command_[i].ref_val =
                std::clamp(wheel_vel + diff, -kMaxWheelSpeed, kMaxWheelSpeed);
            run_command_[i].kp_scale = 0.0;
            run_command_[i].kv_scale = 20.0;
        } else {
            run_command_[i].motor_state = MotorState::POSITION;
            run_command_[i].ref_val = config.axes[i].initial_position;
            run_command_[i].kp_scale = 1.0;
            run_command_[i].kv_scale = 1.0;
        }
    }

    return run_command_;
}

