#pragma once
#include "kalman_filter/kalman_filter.h"

// ボディ並進状態推定KF
// 状態: x = [p, v]  (位置, 速度)
// 入力: u = [ax_forward]  (重力除去済み前方加速度)
// 観測: z = [v_body]
//   v_body = r*ω_avg + L*cos(φ)*φ̇  (ホイール+振子合成速度)
class BodyStateEkf {
public:
    static constexpr size_t DIM_X = 2;  // [p, v]
    static constexpr size_t DIM_Z = 1;  // [v_body]

    void Init();
    void Predict(float dt, float ax_forward);
    void Correct(float v_body);

    float est_position() const { return kf_.vecX()(0); }
    float est_velocity() const { return kf_.vecX()(1); }

private:
    kf::KalmanFilter<DIM_X, DIM_Z> kf_;
    kf::Matrix<DIM_X, DIM_X> matQ_;
    kf::Matrix<DIM_Z, DIM_Z> matR_;
};
