#pragma once
#include <string>
#include <vector>
#include "controller.hpp"
#include "body_state_ekf.hpp"

// 3状態1入力LQRバランスコントローラ（2DOFモデル）
// 状態: X = [ṡ, φ, φ̇]
// 制御: T_φ = TL + TR → TL = TR = T_φ / 2
// ṡ はEKFで推定、φ/φ̇ はIMU直読み
class LqrController : public Controller {
public:
    explicit LqrController(const RobotConfig& config);

    void Reset() override;
    void Update(const std::vector<AxisAct>& motor_status,
                const ImuData& imu_data) override;
    std::vector<AxisRef> Compute(const RobotConfig& config) override;
    EstimatedState EstState() const override;

private:
    static constexpr int kNumStates = 3;

    // MuJoCo導出パラメータ
    static constexpr double kWheelRadius = 0.07795;  // [m]
    static constexpr double kCoMHeight = 0.2484;     // [m]
    static constexpr double kPitchOffset = 0.025;    // [rad] 平衡ピッチ角
    static constexpr double kTickSec = 0.003;        // [s]

    // LQRゲイン K (1×3): CSVから読み込み
    // 状態: [ṡ, φ, φ̇]
    double K_[kNumStates] = {};
    static bool LoadGainCsv(const std::string& path, double K[3]);

    static constexpr double kMaxTorque = 14.0;   // [Nm]
    static constexpr double kKvScale = 20.0;     // 速度サーボゲインスケール（PIDと同値）
    static constexpr double kBaseKv = 0.5 / (2.0 * 3.14159265358979);  // moteus base kv [Nm/(rad/s)]

    // ホイール軸インデックス
    size_t wheel_r_ = SIZE_MAX;
    size_t wheel_l_ = SIZE_MAX;

    // EKFによるボディ並進状態推定 [p, v]
    BodyStateEkf ekf_;

    // IMU直読み
    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;

    std::vector<AxisAct> motor_status_;
    std::vector<AxisRef> run_command_;
};
