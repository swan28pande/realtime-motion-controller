// Discrete PID with derivative on measurement, a first-order derivative filter,
// output clamping and conditional-integration anti-windup.
#pragma once
#include <algorithm>

struct PidGains {
    double kp = 20.0, ki = 10.0, kd = 1.2;
    double out_limit = 2.0;  // actuator limit (Nm)
    double d_filter_hz = 100.0;
};

class Pid {
public:
    explicit Pid(PidGains g = {}) : g_(g) {}

    double update(double setpoint, double measured, double dt) {
        const double err = setpoint - measured;
        // Derivative of the measurement (not the error) avoids a kick on setpoint steps.
        const double raw_d = first_ ? 0.0 : -(measured - prev_meas_) / dt;
        const double alpha = dt / (dt + 1.0 / (2.0 * 3.14159265358979 * g_.d_filter_hz));
        d_ += alpha * (raw_d - d_);
        prev_meas_ = measured;
        first_ = false;

        const double unsat = g_.kp * err + integ_ + g_.kd * d_;
        const double out = std::clamp(unsat, -g_.out_limit, g_.out_limit);
        // Only integrate when not saturated, or when integrating pulls away from the limit.
        if (unsat == out || (unsat > out) != (err > 0)) integ_ += g_.ki * err * dt;
        integ_ = std::clamp(integ_, -g_.out_limit, g_.out_limit);
        return out;
    }

    void reset() { integ_ = d_ = 0.0; first_ = true; }

private:
    PidGains g_;
    double integ_ = 0.0, d_ = 0.0, prev_meas_ = 0.0;
    bool first_ = true;
};
