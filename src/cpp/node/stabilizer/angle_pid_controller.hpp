#pragma once
#include <vector>
#include "controller.hpp"
#include "../../lib/pid.hpp"
#include "kalman_filter/kalman_filter.h"
#include "posture_ik.hpp"

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
    // 内側ループのゲインは重心高さでスケジューリングする:
    // h_cmd_ を [h0, h0+kMaxComRise] で 0..1 に正規化し Min→Max を線形補間
    // (posture IK 無効時は常に Min = 従来どおり)。振子は高いほど復元・減衰項が
    // 1/L で薄まるので、ゲインを高さに比例して上げるのが物理的に自然。
    // Max の初期値は L 比 (h0≈0.15 → +0.08 で ×1.5) からの当たり — 実機で調整する
    static constexpr double kAngleKpMin = 60.0;
    static constexpr double kAngleKpMax = 95.0;
    static constexpr double kAngleKiMin = 400.0;
    static constexpr double kAngleKiMax = 555.0;
    static constexpr double kAngleKd = 0.0;
    // I 項の積分クランプ (Min ゲイン時の値)。I 項の出力上限を高さによらず
    // kAngleKiMin * kAngleMaxIntegral = 35 rad/s (≈ kMaxWheelSpeed) に保つため、
    // 実行時は ki に反比例させて縮めて渡す (Ki が上がったのにクランプ据え置きだと
    // 「出力は飽和しているのに積分は増え続ける」領域が再発する)
    static constexpr double kAngleMaxIntegral = 0.1;
    static constexpr double kAngleDDeadZone = 0.03;

    // 外側ループ (速度 → 目標角オフセット)。帰還は EKF の並進速度 [m/s]、指令も [m/s]。
    // 以前はエンコーダ ω [rad/s] だった。ゲインは r=0.07795 で換算して挙動据え置き:
    //   kVelKi 0.01 [rad/(rad)] → 0.01/r = 0.128 [rad/m]、積分クランプ 5.0 [rad] → 5.0*r = 0.39 [m]
    static constexpr double kVelKp = 0.03;            // [rad/(m/s)] 速度 P。EKF が滑らかなので上げられる余地あり
    static constexpr double kVelKi = 0.006;          // [rad/m]  (= 位置帰還)
    static constexpr double kVelMaxIntegral = 0.39;  // [m] = 積分 (走行距離) の上限
    // 外側ループのエンコーダ帰還 ω → m/s の換算半径 (EKF 側の値と同じ。main.cpp 参照)
    static constexpr double kFeedbackWheelRadius = 0.07795;

    static constexpr double kMaxAngleOffset = 0.05;
    static constexpr double kMaxWheelSpeed = 30.0;
    // 走行指令のフルスケール [m/s] (重心の対地速度 = EKF と同じ単位)。送り側は
    // 正規化 -1..1 を送ってくるだけで、物理速度への変換はこの 1 箇所が正。
    // 0.078 m/s = 従来の 1.0 rad/s * r と同等
    static constexpr double kMaxDriveVel = 0.6;
    // 旋回のフルスケール [rad/s]: 左右ホイールに ±この値まで差動で足す。
    // 差動は左右平均から消えるためバランス (ピッチ) と外側ループ (平均速度) の
    // どちらにも一次では影響しない — だからこちらは出力への直接加算でよい
    static constexpr double kMaxYawWheelDiff = 0.5;
    static constexpr double kTickSec = 0.003;
    // 姿勢 (重心高さ) 変更: 右スティックは正規化レート -1..1 で来る。
    // フルスケール [m/s] と、初期姿勢からの上昇量の上限 [m] はここが正。
    // 上限 0.08 m は mimic_v2_5 で hip の可動域下限 (0.6 rad) に当たるところ
    // (scripts/mjmodel_converter/README.md)。モデルの関節リミットでも別途クランプする
    static constexpr double kMaxComHeightVel = 0.015;
    static constexpr double kMaxComRise = 0.08;
    static constexpr double kPostureMaxStep = 0.02;   // IK 1 反復あたりの関節角ステップ上限 [rad]

    Pid angle_pid_;
    Pid velocity_pid_;

    size_t wheel_r_ = SIZE_MAX;
    size_t wheel_l_ = SIZE_MAX;

    std::vector<AxisAct> motor_status_;
    double pitch_ = 0.0;
    double pitch_rate_ = 0.0;
    double est_velocity_ = 0.0;    // EKF の並進速度 [m/s] (外側ループの帰還)
    double drive_forward_ = 0.0;   // 外側ループの目標並進速度 [m/s]
    double drive_yaw_ = 0.0;       // 左右ホイールへの差動 [rad/s] (右旋回が正のつもり)
    std::vector<AxisRef> run_command_;

    // 姿勢 (重心高さ) 変更。posture_ik_ が読めていれば有効 (robot_config の model_mjcf が必要。
    // MJCF の joint 名は軸名と同じ)。無効なら hip/knee は従来どおり initial_position 固定
    bool posture_enabled_ = false;
    PostureIk posture_ik_;
    size_t hip_r_ = SIZE_MAX, knee_r_ = SIZE_MAX, hip_l_ = SIZE_MAX, knee_l_ = SIZE_MAX;
    double hip_init_ = 0.0, knee_init_ = 0.0;   // initial_position (左右同じ前提)
    double h0_ = 0.0;              // 初期姿勢での重心高さ (車軸基準) [m] = 指令の下限
    double h_cmd_ = 0.0;           // 目標重心高さ [m]
    double height_rate_ = 0.0;     // [m/s]
    double hip_cmd_ = 0.0, knee_cmd_ = 0.0;   // IK 解 (左右同じ値を出す)
    int posture_log_countdown_ = 0;
};
