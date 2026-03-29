#pragma once
#include "kalman_filter/kalman_filter.h"

// ボディ状態推定EKF
// 状態: x = [ṡ, α, α̇]  (前進速度, ヨー角, ヨーレート)
// 入力: u = [ax_forward]  (重力除去済み前方加速度)
// 観測: z = [v_body, gz_imu, yaw_diff_wheel]
//   z0: ボディ速度 = r*ω_avg + L*cos(φ)*φ̇
//   z1: IMUジャイロ gz
//   z2: 差動ホイール速度 r*(ω_R - ω_L) / D
//
// φ, φ̇ はIMU直読み（推定対象外）
class BodyStateEkf {
public:
    static constexpr size_t DIM_X = 3;  // [ṡ, α, α̇]
    static constexpr size_t DIM_Z = 3;  // [v_body, gz, yaw_wheel]

    void Init();
    void Predict(float dt, float ax_forward);
    void Correct(float v_body, float gz_imu, float yaw_rate_wheel);

    float est_velocity() const { return kf_.vecX()(0); }
    float est_yaw() const { return kf_.vecX()(1); }
    float est_yaw_rate() const { return kf_.vecX()(2); }

    void ResetYaw() {
        kf_.vecX()(1) = 0.0F;  // ヨー角リセット
    }

private:
    kf::KalmanFilter<DIM_X, DIM_Z> kf_;
    kf::Matrix<DIM_X, DIM_X> matQ_;  // プロセスノイズ
    kf::Matrix<DIM_Z, DIM_Z> matR_;  // 観測ノイズ
};
