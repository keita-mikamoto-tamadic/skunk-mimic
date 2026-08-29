#pragma once
#include <cmath>
#include <algorithm>

class Pid {
public:
    Pid() = default;
    Pid(double kp, double ki, double kd,
        double max_integral, double d_dead_zone = 0.0)
        : kp_(kp), ki_(ki), kd_(kd),
          max_integral_(max_integral), d_dead_zone_(d_dead_zone) {}

    void Reset() { integral_ = 0.0; }

    // ゲインスケジューリング用: 実行中にゲイン/積分クランプを差し替える。
    // integral_ (∫e) は保持したまま。ki の変更は I 項出力を ki 比でスケール
    // するが、呼び出し側が遅いレートで変える前提 (不連続にしない)
    void SetGains(double kp, double ki, double kd, double max_integral) {
        kp_ = kp; ki_ = ki; kd_ = kd; max_integral_ = max_integral;
    }

    // error: 制御偏差
    // d_input: 外部微分入力（IMU角速度など。0ならD項無効）
    // dt: 制御周期 [s]
    double Compute(double error, double d_input, double dt) {
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -max_integral_, max_integral_);

        double d_term = (std::abs(d_input) < d_dead_zone_) ? 0.0 : d_input;

        return kp_ * error + ki_ * integral_ + kd_ * d_term;
    }

private:
    double kp_ = 0, ki_ = 0, kd_ = 0;
    double max_integral_ = 0;
    double d_dead_zone_ = 0;
    double integral_ = 0;
};
