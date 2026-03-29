#include "body_state_ekf.hpp"

void BodyStateEkf::Init() {
    // 初期状態: 静止
    kf_.vecX() << 0.0F, 0.0F, 0.0F;

    // 初期共分散: 不確かさ大
    kf_.matP() = kf::Matrix<DIM_X, DIM_X>::Identity() * 1.0F;

    // プロセスノイズ Q
    //          ṡ      α      α̇
    matQ_ << 0.1F,  0.0F,  0.0F,
             0.0F,  0.01F, 0.0F,
             0.0F,  0.0F,  0.1F;

    // 観測ノイズ R
    //          v_body  gz_imu  yaw_wheel
    matR_ << 0.01F, 0.0F,  0.0F,
             0.0F,  0.01F, 0.0F,
             0.0F,  0.0F,  0.05F;  // ホイールオドメトリは滑り有り→やや不信頼
}

void BodyStateEkf::Predict(float dt, float ax_forward) {
    // 状態遷移: CV + 加速度入力
    // ṡ_new = ṡ + ax * dt
    // α_new = α + α̇ * dt
    // α̇_new = α̇  (定常ヨーレートモデル)
    auto stateTransitionFunc = [&](const kf::Vector<DIM_X>& state) -> kf::Vector<DIM_X> {
        kf::Vector<DIM_X> next;
        next(0) = state(0) + ax_forward * dt;
        next(1) = state(1) + state(2) * dt;
        next(2) = state(2);
        return next;
    };

    // ヤコビアン F = df/dx
    kf::Matrix<DIM_X, DIM_X> matF;
    matF << 1.0F, 0.0F, 0.0F,
            0.0F, 1.0F,   dt,
            0.0F, 0.0F, 1.0F;

    kf_.predictEkf(stateTransitionFunc, matF, matQ_);
}

void BodyStateEkf::Correct(float v_body, float gz_imu, float yaw_rate_wheel) {
    // 観測ベクトル
    kf::Vector<DIM_Z> vecZ;
    vecZ << v_body, gz_imu, yaw_rate_wheel;

    // 観測関数 h(x)
    // z0 = ṡ         (ボディ速度を直接観測)
    // z1 = α̇         (ジャイロがヨーレートを観測)
    // z2 = α̇         (差動ホイールもヨーレートを観測)
    auto measurementFunc = [](const kf::Vector<DIM_X>& state) -> kf::Vector<DIM_Z> {
        kf::Vector<DIM_Z> z_pred;
        z_pred(0) = state(0);  // ṡ
        z_pred(1) = state(2);  // α̇
        z_pred(2) = state(2);  // α̇
        return z_pred;
    };

    // ヤコビアン H = dh/dx
    kf::Matrix<DIM_Z, DIM_X> matH;
    matH << 1.0F, 0.0F, 0.0F,   // v_body → ṡ
            0.0F, 0.0F, 1.0F,   // gz     → α̇
            0.0F, 0.0F, 1.0F;   // wheel  → α̇

    kf_.correctEkf(measurementFunc, vecZ, matR_, matH);
}
