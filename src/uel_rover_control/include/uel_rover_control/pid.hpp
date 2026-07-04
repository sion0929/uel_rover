#pragma once
#include <algorithm>

class PID {
public:
    PID(double kp, double ki, double kd, double out_min, double out_max)
        : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

    double compute(double target, double actual, double dt) {
        if (dt <= 0.0) return 0.0;
        double error      = target - actual;
        integral_        += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_       = error;
        return std::clamp(kp_ * error + ki_ * integral_ + kd_ * derivative, out_min_, out_max_);
    }

    void reset() { integral_ = 0.0; prev_error_ = 0.0; }

    void setGains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
        reset();
    }

private:
    double kp_, ki_, kd_;
    double out_min_, out_max_;
    double integral_   = 0.0;
    double prev_error_ = 0.0;
};
