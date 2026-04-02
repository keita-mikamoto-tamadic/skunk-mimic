#include "body_state_ekf.hpp"

void BodyStateEkf::Init() {
    kf_.vecX() << 0.0F, 0.0F;
    kf_.matP() = kf::Matrix<DIM_X, DIM_X>::Identity() * 1.0F;

    // プロセスノイズ Q
    //          p      v
    matQ_ << 0.01F, 0.0F,
             0.0F,  0.1F;

    // 観測ノイズ R
    matR_ << 0.01F;
}

void BodyStateEkf::Predict(float dt, float ax_forward) {
    // p_new = p + v * dt + 0.5 * a * dt^2
    // v_new = v + a * dt
    auto stateTransitionFunc = [&](const kf::Vector<DIM_X>& state) -> kf::Vector<DIM_X> {
        kf::Vector<DIM_X> next;
        next(0) = state(0) + state(1) * dt + 0.5F * ax_forward * dt * dt;
        next(1) = state(1) + ax_forward * dt;
        return next;
    };

    kf::Matrix<DIM_X, DIM_X> matF;
    matF << 1.0F,   dt,
            0.0F, 1.0F;

    kf_.predictEkf(stateTransitionFunc, matF, matQ_);
}

void BodyStateEkf::Correct(float v_body) {
    kf::Vector<DIM_Z> vecZ;
    vecZ << v_body;

    // h(x) = v = x(1)
    auto measurementFunc = [](const kf::Vector<DIM_X>& state) -> kf::Vector<DIM_Z> {
        kf::Vector<DIM_Z> z_pred;
        z_pred(0) = state(1);
        return z_pred;
    };

    kf::Matrix<DIM_Z, DIM_X> matH;
    matH << 0.0F, 1.0F;

    kf_.correctEkf(measurementFunc, vecZ, matR_, matH);
}
