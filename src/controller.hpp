// Motion controller: a 1 kHz real-time PID loop per axis, fed by encoder
// feedback from CAN, sending torque commands back over CAN.
#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "can_bus.hpp"
#include "pid.hpp"

struct AxisState {
    std::atomic<double> setpoint{0};  // rad
    std::atomic<double> position{0};  // rad, from encoder
    std::atomic<double> velocity{0};  // rad/s
    std::atomic<double> effort{0};    // Nm
    std::atomic<int64_t> last_feedback_ns{0};
};

struct LoopStats {
    std::atomic<int64_t> cycles{0};
    std::atomic<int64_t> max_late_ns{0};
};

class Controller {
public:
    Controller(CanBus& bus, int num_axes, int rate_hz = 1000, PidGains gains = {}, int counts_per_rev = 4096);

    void rx_loop(const std::atomic<bool>& running);       // CAN receive thread
    void control_loop(const std::atomic<bool>& running);  // real-time PID thread

    void set_setpoint_deg(int axis, double deg);  // axis is 1-based
    int num_axes() const { return static_cast<int>(axes_.size()); }
    const AxisState& axis(int i) const { return *axes_[i]; }
    const LoopStats& stats() const { return stats_; }
    int rate_hz() const { return rate_hz_; }

private:
    CanBus& bus_;
    std::vector<std::unique_ptr<AxisState>> axes_;
    std::vector<Pid> pids_;
    int rate_hz_;
    int counts_per_rev_;
    LoopStats stats_;
};
