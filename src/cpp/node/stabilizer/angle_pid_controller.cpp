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

    // 姿勢 IK: robot_config に model_mjcf があれば有効化 (MJCF の joint 名 = 軸名)
    for (size_t i = 0; i < axis_count; i++) {
        if (config.axes[i].name == "hip_pitch_r") hip_r_ = i;
        if (config.axes[i].name == "knee_r")      knee_r_ = i;
        if (config.axes[i].name == "hip_pitch_l") hip_l_ = i;
        if (config.axes[i].name == "knee_l")      knee_l_ = i;
    }
    const bool leg_axes_ok = hip_r_ != SIZE_MAX && knee_r_ != SIZE_MAX &&
                             hip_l_ != SIZE_MAX && knee_l_ != SIZE_MAX &&
                             wheel_r_ != SIZE_MAX && wheel_l_ != SIZE_MAX;
    if (config.model_mjcf.empty()) {
        std::cout << "posture IK: disabled (no model_mjcf in robot_config)" << std::endl;
    } else if (!leg_axes_ok) {
        std::cout << "posture IK: disabled (hip_pitch_*/knee_*/wheel_* axes not all present)" << std::endl;
    } else {
        // MJCF の駆動 joint 名 = robot_config の軸名 (mjmodel_converter の約束)
        PostureIk::Config pc;
        pc.mjcf_path = robot_config::ResolveRepoPath(config.model_mjcf);
        pc.hip_r  = config.axes[hip_r_].name;
        pc.knee_r = config.axes[knee_r_].name;
        pc.hip_l  = config.axes[hip_l_].name;
        pc.knee_l = config.axes[knee_l_].name;
        pc.wheel_r = config.axes[wheel_r_].name;
        pc.wheel_l = config.axes[wheel_l_].name;
        std::string err;
        if (!posture_ik_.Load(pc, &err)) {
            std::cerr << "posture IK: disabled (" << err << ")" << std::endl;
        } else {
            hip_init_  = config.axes[hip_r_].initial_position;
            knee_init_ = config.axes[knee_r_].initial_position;
            if (std::abs(config.axes[hip_l_].initial_position - hip_init_) > 1e-6 ||
                std::abs(config.axes[knee_l_].initial_position - knee_init_) > 1e-6) {
                std::cerr << "posture IK: WARNING left/right initial_position differ; using right" << std::endl;
            }
            const Eigen::Vector2d c0 = posture_ik_.ComRelAxle(hip_init_, knee_init_, kTargetPitch);
            h0_ = c0.y();
            h_cmd_ = h0_;
            hip_cmd_ = hip_init_;
            knee_cmd_ = knee_init_;
            posture_enabled_ = true;
            std::cout << "posture IK: enabled (" << pc.mjcf_path << ", mass "
                      << posture_ik_.total_mass() << " kg; at initial pose CoM is "
                      << c0.x() * 1000.0 << " mm ahead of the axle, " << h0_ << " m above it;"
                      << " hip range [" << posture_ik_.hip_min() << ", " << posture_ik_.hip_max()
                      << "], knee range [" << posture_ik_.knee_min() << ", " << posture_ik_.knee_max()
                      << "]; h range [" << h0_ << ", " << h0_ + kMaxComRise << "])" << std::endl;
        }
    }

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
    // RUN 開始時は READY で initial_position に来ているので、姿勢もそこから
    height_rate_ = 0.0;
    h_cmd_ = h0_;
    hip_cmd_ = hip_init_;
    knee_cmd_ = knee_init_;
}

void AnglePidController::SetDriveCommand(const DriveCommand& cmd) {
    // forward/yaw/height は正規化 -1..1。クランプしてからフルスケールを掛ける
    drive_forward_ = std::clamp(cmd.forward, -1.0, 1.0) * kMaxDriveWheelVel;
    drive_yaw_     = std::clamp(cmd.yaw,     -1.0, 1.0) * kMaxYawWheelDiff;
    height_rate_   = std::clamp(cmd.height,  -1.0, 1.0) * kMaxComHeightVel;
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

    // 姿勢 (重心高さ): スティックのレートを積分して目標高さにし、倒立点 (kTargetPitch で
    // 重心が車軸の真上) を保つ hip/knee を IK で 1 反復ずつ追従させる。外側ループの
    // angle_offset は過渡補正なので IK の基準には使わない (幾何は kTargetPitch で固定)
    if (posture_enabled_) {
        h_cmd_ = std::clamp(h_cmd_ + height_rate_ * kTickSec, h0_, h0_ + kMaxComRise);
        const double resid = posture_ik_.Step(kTargetPitch, h_cmd_, hip_cmd_, knee_cmd_, kPostureMaxStep);
        if (height_rate_ != 0.0 && --posture_log_countdown_ <= 0) {
            posture_log_countdown_ = 333;   // ~1 s
            std::cout << "posture: h=" << h_cmd_ << " (+" << h_cmd_ - h0_ << ") hip=" << hip_cmd_
                      << " knee=" << knee_cmd_ << " resid=" << resid * 1000.0 << "mm" << std::endl;
        }
    }

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
            double ref = config.axes[i].initial_position;
            if (posture_enabled_) {
                if (i == hip_r_ || i == hip_l_)   ref = hip_cmd_;
                if (i == knee_r_ || i == knee_l_) ref = knee_cmd_;
            }
            run_command_[i].ref_val = ref;
            run_command_[i].kp_scale = 1.0;
            run_command_[i].kv_scale = 1.0;
        }
    }

    return run_command_;
}

