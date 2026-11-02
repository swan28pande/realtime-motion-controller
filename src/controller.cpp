#include "controller.hpp"

#include <cmath>
#include <cstdio>

#include "rt_thread.hpp"

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr int64_t kFeedbackTimeoutNs = 50'000'000;  // 50 ms without feedback -> stop driving that axis
}  // namespace

Controller::Controller(CanBus& bus, int num_axes, int rate_hz, PidGains gains, int counts_per_rev)
    : bus_(bus), pids_(num_axes, Pid(gains)), rate_hz_(rate_hz), counts_per_rev_(counts_per_rev) {
    for (int i = 0; i < num_axes; ++i) axes_.push_back(std::make_unique<AxisState>());
}

void Controller::set_setpoint_deg(int axis, double deg) {
    if (axis >= 1 && axis <= num_axes()) axes_[axis - 1]->setpoint = deg * kDeg2Rad;
}

void Controller::rx_loop(const std::atomic<bool>& running) {
    const int n = num_axes();
    while (running) {
        CanFrame f;
        if (!bus_.recv(f, 20)) continue;
        if (f.id > canid::kFeedback && f.id <= canid::kFeedback + n && f.dlc >= 8) {
            AxisState& a = *axes_[f.id - canid::kFeedback - 1];
            a.position = get<int32_t>(f, 0) * 2.0 * M_PI / counts_per_rev_;
            a.velocity = get<float>(f, 4);
            a.last_feedback_ns = now_ns();
        } else if (f.id > canid::kSetpoint && f.id <= canid::kSetpoint + n && f.dlc >= 4) {
            set_setpoint_deg(static_cast<int>(f.id - canid::kSetpoint), get<float>(f, 0));
        }
    }
}

void Controller::control_loop(const std::atomic<bool>& running) {
    const double dt = 1.0 / rate_hz_;
    PeriodicTimer timer(1'000'000'000LL / rate_hz_);
    while (running) {
        const int64_t late = timer.wait();
        if (late > stats_.max_late_ns) stats_.max_late_ns = late;
        const int64_t now = now_ns();
        for (int i = 0; i < num_axes(); ++i) {
            AxisState& a = *axes_[i];
            double u = 0.0;
            if (now - a.last_feedback_ns < kFeedbackTimeoutNs) {
                u = pids_[i].update(a.setpoint, a.position, dt);
            } else {
                pids_[i].reset();  // stale feedback: command zero torque
            }
            a.effort = u;
            CanFrame cmd;
            cmd.id = canid::kCommand + (i + 1);
            cmd.dlc = 4;
            put<float>(cmd, 0, static_cast<float>(u));
            bus_.send(cmd);
        }
        ++stats_.cycles;
    }
}
