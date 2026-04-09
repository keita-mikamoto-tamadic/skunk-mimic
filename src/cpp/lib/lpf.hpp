#pragma once

// 1次IIRローパスフィルタ
// α = 2π×fc×dt / (1 + 2π×fc×dt)
class LowPassFilter {
public:
    LowPassFilter() = default;
    explicit LowPassFilter(double cutoff_hz, double dt)
        : alpha_(2.0 * 3.14159265358979 * cutoff_hz * dt
                / (1.0 + 2.0 * 3.14159265358979 * cutoff_hz * dt)) {}

    double Update(double input) {
        if (!initialized_) {
            y_ = input;
            initialized_ = true;
            return y_;
        }
        y_ += alpha_ * (input - y_);
        return y_;
    }

    void Reset() {
        initialized_ = false;
        y_ = 0.0;
    }

    double value() const { return y_; }

private:
    double alpha_ = 0.0;
    double y_ = 0.0;
    bool initialized_ = false;
};
