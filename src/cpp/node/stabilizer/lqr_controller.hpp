#pragma once
#include <string>
#include <vector>
#include "controller.hpp"

// 3状態1入力LQRバランスコントローラ（2DOFモデル）
// 状態: X = [ṡ, φ, φ̇]
// 制御: T_φ = TL + TR → 速度指令に変換して送信
// ṡ はmain.cppのEKFから受け取る、φ/φ̇ はIMU直読み
class LqrController : public Controller {
public:
    explicit LqrController(const RobotConfig& config);

    void Reset() override;
    void Update(const std::vector<AxisAct>& motor_status,
                const ImuData& imu_data,
                const BodyStateEkf& ekf) override;
    std::vector<AxisRef> Compute(const RobotConfig& config) override;

private:
    static constexpr int kNumStates = 3;

    // MuJoCo導出パラメータ
    static constexpr double kWheelRadius = 0.07795;  // [m]
    static constexpr double kCoMHeight = 0.2484;     // [m]
    static constexpr double kPitchOffset = 0.025;    // [rad] 平衡ピッチ角

    // LQRゲイン K (1×3): CSVから読み込み
    double K_[kNumStates] = {};
    static bool LoadGainCsv(const std::string& path, double K[3]);

    static constexpr double kMaxTorque = 14.0;   // [Nm]
    static constexpr double kKvScale = 20.0;     // 速度サーボゲインスケール
    static constexpr double kBaseKv = 0.5 / (2.0 * 3.14159265358979);  // moteus base kv

    // ホイール軸インデックス
    size_t wheel_r_ = SIZE_MAX;
    size_t wheel_l_ = SIZE_MAX;

    // 状態（Update で更新）
    double est_velocity_ = 0.0;  // EKFからの推定速度
    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;

    std::vector<AxisAct> motor_status_;
    std::vector<AxisRef> run_command_;
};
