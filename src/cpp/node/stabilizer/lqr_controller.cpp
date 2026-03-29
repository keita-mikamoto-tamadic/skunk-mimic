#include "lqr_controller.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

constexpr const char* kGainCsvPath = "scripts/calc_lqr_gain/lqrGain.csv";

bool LqrController::LoadGainCsv(const std::string& path,
                                 double K[kNumInputs][kNumStates]) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    for (int row = 0; row < kNumInputs; row++) {
        std::string line;
        if (!std::getline(file, line)) return false;
        std::istringstream ss(line);
        for (int col = 0; col < kNumStates; col++) {
            std::string token;
            if (!std::getline(ss, token, ',')) return false;
            K[row][col] = std::stod(token);
        }
    }
    return true;
}

LqrController::LqrController(const RobotConfig& config) {
    const size_t axis_count = config.axes.size();

    for (size_t i = 0; i < axis_count; i++) {
        if (config.axes[i].name == "wheel_r") wheel_r_ = i;
        if (config.axes[i].name == "wheel_l") wheel_l_ = i;
    }
    if (wheel_r_ == SIZE_MAX || wheel_l_ == SIZE_MAX) {
        std::cerr << "wheel axes not found" << std::endl;
    }
    std::cout << "LQR: wheel_r=" << wheel_r_
              << " wheel_l=" << wheel_l_ << std::endl;

    // ゲインCSV読み込み
    if (!LoadGainCsv(kGainCsvPath, K_)) {
        std::cerr << "failed to load " << kGainCsvPath << std::endl;
    } else {
        std::cout << "LQR gain loaded (2×4):" << std::endl;
        for (int i = 0; i < kNumInputs; i++) {
            std::cout << "  K[" << i << "]: ";
            for (int j = 0; j < kNumStates; j++)
                std::cout << K_[i][j] << " ";
            std::cout << std::endl;
        }
    }

    // EKF初期化
    ekf_.Init();

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

void LqrController::Reset() {
    ekf_.Init();  // EKF状態リセット
}

void LqrController::Update(const std::vector<AxisAct>& motor_status,
                            const ImuData& imu_data) {
    motor_status_ = motor_status;
    pitch_ = imu_data.pitch;
    pitch_rate_ = imu_data.gy;

    // --- EKF predict: 重力除去済み前方加速度 ---
    double ax_forward = imu_data.ax * std::cos(pitch_)
                      + imu_data.az * std::sin(pitch_);
    ekf_.Predict(static_cast<float>(kTickSec),
                 static_cast<float>(ax_forward));

    // --- EKF correct: 3つの観測値 ---
    // z0: ボディ速度 = r*ω_avg + L*cos(φ)*φ̇
    double wheel_vel_avg =
        (motor_status[wheel_r_].velocity + motor_status[wheel_l_].velocity) / 2.0;
    double v_body = kWheelRadius * wheel_vel_avg
                  + kCoMHeight * std::cos(pitch_) * pitch_rate_;

    // z1: IMUジャイロ gz
    double gz_imu = imu_data.gz;

    // z2: 差動ホイール速度 → ヨーレート = r*(ω_R - ω_L) / D
    double yaw_rate_wheel = kWheelRadius
        * (motor_status[wheel_r_].velocity - motor_status[wheel_l_].velocity)
        / kTrackWidth;

    ekf_.Correct(static_cast<float>(v_body),
                 static_cast<float>(gz_imu),
                 static_cast<float>(yaw_rate_wheel));
}

std::vector<AxisRef> LqrController::Compute(const RobotConfig& config) {
    // 状態ベクトル X = [ṡ, φ, φ̇, α̇]
    // ṡ, α̇ はEKF推定値、φ, φ̇ はIMU直読み
    double X[kNumStates] = {
        static_cast<double>(ekf_.est_velocity()),
        pitch_ - kPitchOffset,
        pitch_rate_,
        static_cast<double>(ekf_.est_yaw_rate()),
    };

    // u = -K · X → [T_φ, T_α]
    double t_phi = 0.0;
    double t_alpha = 0.0;
    for (int j = 0; j < kNumStates; j++) {
        t_phi   += -K_[0][j] * X[j];
        t_alpha += -K_[1][j] * X[j];
    }

    // T_φ, T_α → 左右ホイールトルクに分配
    // TL = T_φ/2 - T_α · D/(2r)
    // TR = T_φ/2 + T_α · D/(2r)
    double scale = kTrackWidth / (2.0 * kWheelRadius);
    double torque_l = t_phi / 2.0 - t_alpha * scale;
    double torque_r = t_phi / 2.0 + t_alpha * scale;

    torque_l = std::clamp(torque_l, -kMaxTorque, kMaxTorque);
    torque_r = std::clamp(torque_r, -kMaxTorque, kMaxTorque);

    const size_t axis_count = config.axes.size();
    for (size_t i = 0; i < axis_count; i++) {
        if (i == wheel_l_) {
            run_command_[i].motor_state = MotorState::TORQUE;
            run_command_[i].ref_val = torque_l;
            run_command_[i].kp_scale = 0.0;
            run_command_[i].kv_scale = 0.0;
        } else if (i == wheel_r_) {
            run_command_[i].motor_state = MotorState::TORQUE;
            run_command_[i].ref_val = torque_r;
            run_command_[i].kp_scale = 0.0;
            run_command_[i].kv_scale = 0.0;
        } else {
            run_command_[i].motor_state = MotorState::POSITION;
            run_command_[i].ref_val = config.axes[i].initial_position;
            run_command_[i].kp_scale = 1.0;
            run_command_[i].kv_scale = 1.0;
        }
    }

    return run_command_;
}

EstimatedState LqrController::EstState() const {
    return {
        static_cast<double>(ekf_.est_velocity()),
        static_cast<double>(ekf_.est_yaw()),
        static_cast<double>(ekf_.est_yaw_rate()),
    };
}
