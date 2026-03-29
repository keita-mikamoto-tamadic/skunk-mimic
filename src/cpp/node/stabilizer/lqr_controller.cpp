#include "lqr_controller.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

constexpr const char* kGainCsvPath = "scripts/calc_lqr_gain/lqrGain.csv";

bool LqrController::LoadGainCsv(const std::string& path, double K[3]) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    std::string line;
    if (!std::getline(file, line)) return false;
    std::istringstream ss(line);
    for (int i = 0; i < 3; i++) {
        std::string token;
        if (!std::getline(ss, token, ',')) return false;
        K[i] = std::stod(token);
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
        std::cout << "LQR gain loaded: K=[" << K_[0] << ", "
                  << K_[1] << ", " << K_[2] << "]" << std::endl;
    }

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
    xb_dot_ = 0.0;
}

void LqrController::Update(const std::vector<AxisAct>& motor_status,
                            const ImuData& imu_data) {
    motor_status_ = motor_status;
    pitch_ = imu_data.pitch;
    pitch_rate_ = imu_data.gy;

    // ボディ速度 = ホイール速度 + 振子成分
    double wheel_vel_avg =
        (motor_status[wheel_r_].velocity + motor_status[wheel_l_].velocity) / 2.0;
    xb_dot_ = kWheelRadius * wheel_vel_avg + kCoMHeight * std::cos(pitch_) * pitch_rate_;
}

std::vector<AxisRef> LqrController::Compute(const RobotConfig& config) {
    // 状態ベクトル X = [ẋb, φ, φ̇]
    double X[3] = {xb_dot_, pitch_, pitch_rate_};

    // T_φ = K * X（トータルトルク）
    double t_phi = 0.0;
    for (int j = 0; j < 3; j++) {
        t_phi += -K_[j] * X[j];
    }

    // 各ホイールに均等配分
    double torque = t_phi / 2.0;
    torque = std::clamp(torque, -kMaxTorque, kMaxTorque);

    const size_t axis_count = config.axes.size();
    for (size_t i = 0; i < axis_count; i++) {
        if (i == wheel_l_ || i == wheel_r_) {
            run_command_[i].motor_state = MotorState::TORQUE;
            run_command_[i].ref_val = torque;
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

double LqrController::EstBodyVel() const {
    return xb_dot_;
}
