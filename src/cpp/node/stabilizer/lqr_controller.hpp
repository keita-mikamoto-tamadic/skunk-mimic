#pragma once
#include <string>
#include <vector>
#include "controller.hpp"
#include "body_state_ekf.hpp"

// 4状態2入力LQRバランス+ステアリングコントローラ
// 状態: X = [ṡ, φ, φ̇, α̇]
// 制御: u = [T_φ, T_α] → TL = T_φ/2 - T_α*D/(2r), TR = T_φ/2 + T_α*D/(2r)
// 状態推定: EKF で [ṡ, α, α̇] を推定、φ/φ̇ はIMU直読み
class LqrController : public Controller {
public:
    explicit LqrController(const RobotConfig& config);

    void Reset() override;
    void Update(const std::vector<AxisAct>& motor_status,
                const ImuData& imu_data) override;
    std::vector<AxisRef> Compute(const RobotConfig& config) override;
    EstimatedState EstState() const override;

private:
    static constexpr int kNumStates = 4;
    static constexpr int kNumInputs = 2;

    // MuJoCo導出パラメータ（scripts/calc_lqr_gain で算出）
    static constexpr double kWheelRadius = 0.07795;  // [m]
    static constexpr double kCoMHeight = 0.2484;     // [m] ホイール接地点〜ボディCoM
    static constexpr double kTrackWidth = 0.255;     // [m] トレッド幅 D
    static constexpr double kPitchOffset = 0.025;    // [rad] 平衡ピッチ角
    static constexpr double kTickSec = 0.003;        // [s]

    // LQRゲイン K (2×4): CSVから読み込み
    double K_[kNumInputs][kNumStates] = {};
    static bool LoadGainCsv(const std::string& path,
                            double K[kNumInputs][kNumStates]);

    static constexpr double kMaxTorque = 14.0;   // [Nm] ホイールトルク制限

    // ホイール軸インデックス
    size_t wheel_r_ = SIZE_MAX;
    size_t wheel_l_ = SIZE_MAX;

    // EKFによる状態推定 [ṡ, α, α̇]
    BodyStateEkf ekf_;

    // IMU直読み（EKF推定対象外）
    double pitch_ = 0.0;       // ピッチ角 [rad]
    double pitch_rate_ = 0.0;  // ピッチ角速度 [rad/s]

    std::vector<AxisAct> motor_status_;
    std::vector<AxisRef> run_command_;
};
