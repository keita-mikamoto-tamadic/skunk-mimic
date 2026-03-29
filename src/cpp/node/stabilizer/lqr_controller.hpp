#pragma once
#include <string>
#include <vector>
#include "controller.hpp"

// 3状態LQRバランスコントローラ（論文 式(28)）
// 状態: X = [ẋb, φ, φ̇]
// 制御: T_φ = TL + TR （トータルトルク）→ TL = TR = T_φ / 2
class LqrController : public Controller {
public:
    explicit LqrController(const RobotConfig& config);

    void Reset() override;
    void Update(const std::vector<AxisAct>& motor_status,
                const ImuData& imu_data) override;
    std::vector<AxisRef> Compute(const RobotConfig& config) override;
    double EstBodyVel() const override;

private:
    // MuJoCo導出パラメータ（scripts/calc_lqr_gain で算出）
    static constexpr double kWheelRadius = 0.07795;  // [m]
    static constexpr double kCoMHeight = 0.2484;     // [m] ホイール接地点〜ボディCoM

    // LQRゲイン K (1×3): CSVから読み込み
    // 状態: [ẋb, φ, φ̇]
    double K_[3] = {};
    static bool LoadGainCsv(const std::string& path, double K[3]);

    static constexpr double kMaxTorque = 14.0;   // [Nm] ホイールトルク制限

    // ホイール軸インデックス
    size_t wheel_r_ = SIZE_MAX;
    size_t wheel_l_ = SIZE_MAX;

    // 状態推定
    double xb_dot_ = 0.0;      // ボディ速度 [m/s]
    double pitch_ = 0.0;       // ピッチ角 [rad]
    double pitch_rate_ = 0.0;  // ピッチ角速度 [rad/s]

    std::vector<AxisAct> motor_status_;
    std::vector<AxisRef> run_command_;
};
