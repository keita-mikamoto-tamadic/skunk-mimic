#pragma once
#include <vector>
#include "controller.hpp"
#include "../../lib/pid.hpp"
#include "kalman_filter/kalman_filter.h"

// 倒立振子 PID コントローラ
// 外側ループ: 速度PI（倒立点自動調整）
// 内側ループ: 角度PID → ホイール速度指令
class AnglePidController : public Controller {
public:
    explicit AnglePidController(const RobotConfig& config);

    void Reset() override;
    void Update(const std::vector<AxisAct>& motor_status,
                const ImuData& imu_data,
                const BodyStateEkf& ekf) override;
    std::vector<AxisRef> Compute(const RobotConfig& config) override;

private:
    // PID 定数
    static constexpr double kTargetPitch = 0.0465;
    static constexpr double kAngleKp = 12.0;
    static constexpr double kAngleKi = 325.0;
    static constexpr double kAngleKd = 0.17;
    static constexpr double kAngleMaxIntegral = 0.21;
    static constexpr double kAngleDDeadZone = 0.1;

    static constexpr double kVelKp = 0.0001;
    static constexpr double kVelKi = 0.01;
    static constexpr double kVelMaxIntegral = 0.5;

    static constexpr double kMaxAngleOffset = 0.05;
    static constexpr double kMaxWheelSpeed = 30.0;
    static constexpr double kTickSec = 0.003;

    // ボディ速度推定パラメータ（MuJoCo XMLから算出）
    static constexpr double kWheelRadius = 0.05;  // TODO: XMLから正確な値を確認
    static constexpr double kCoMHeight = 0.15;    // TODO: ホイール軸〜重心距離

    Pid angle_pid_;
    Pid velocity_pid_;

    size_t wheel_r_ = SIZE_MAX;
    size_t wheel_l_ = SIZE_MAX;

    std::vector<AxisAct> motor_status_;
    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
    std::vector<AxisRef> run_command_;
};
