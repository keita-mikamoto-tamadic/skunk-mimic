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
    void SetDriveCommand(const DriveCommand& cmd) override;

private:
    // PID 定数
    static constexpr double kTargetPitch = 0.165;
    static constexpr double kAngleKp = 70.0;
    static constexpr double kAngleKi = 350.0;
    static constexpr double kAngleKd = 0.0;
    static constexpr double kAngleMaxIntegral = 0.1;
    static constexpr double kAngleDDeadZone = 0.03;

    static constexpr double kVelKp = 0.0;
    static constexpr double kVelKi = 0.01;
    static constexpr double kVelMaxIntegral = 5.0;

    static constexpr double kMaxAngleOffset = 0.05;
    static constexpr double kMaxWheelSpeed = 30.0;
    // 走行指令のフルスケール [rad/s] (ホイール角速度)。送り側は正規化 -1..1 を
    // 送ってくるだけで、物理速度への変換はこの 1 箇所が正
    static constexpr double kMaxDriveWheelVel = 1.0;
    // 旋回のフルスケール [rad/s]: 左右ホイールに ±この値まで差動で足す。
    // 差動は左右平均から消えるためバランス (ピッチ) と外側ループ (平均速度) の
    // どちらにも一次では影響しない — だからこちらは出力への直接加算でよい
    static constexpr double kMaxYawWheelDiff = 0.5;
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
    double drive_forward_ = 0.0;   // 外側ループの目標ホイール速度 [rad/s]
    double drive_yaw_ = 0.0;       // 左右ホイールへの差動 [rad/s] (右旋回が正のつもり)
    std::vector<AxisRef> run_command_;
};
