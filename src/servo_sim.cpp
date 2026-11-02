#include "servo_sim.hpp"

#include <cmath>

#include "rt_thread.hpp"

ServoSim::ServoSim(CanBus& bus, int num_motors, int feedback_hz, MotorParams p)
    : bus_(bus), motors_(num_motors), feedback_hz_(feedback_hz), p_(p) {}

void ServoSim::run(const std::atomic<bool>& running) {
    constexpr int kSubsteps = 10;  // integrate physics at 10x the feedback rate
    const double dt = 1.0 / (feedback_hz_ * kSubsteps);
    PeriodicTimer timer(1'000'000'000LL / feedback_hz_);
    const int n = static_cast<int>(motors_.size());

    while (running) {
        CanFrame f;
        while (bus_.recv(f, 0)) {  // drain pending torque commands
            int id = static_cast<int>(f.id) - static_cast<int>(canid::kCommand);
            if (id >= 1 && id <= n && f.dlc >= 4) motors_[id - 1].torque = get<float>(f, 0);
        }
        for (int i = 0; i < n; ++i) {
            Motor& m = motors_[i];
            for (int s = 0; s < kSubsteps; ++s) {
                if (std::abs(m.vel) < 1e-3 && std::abs(m.torque) <= p_.coulomb) {
                    m.vel = 0.0;  // stiction: not enough torque to break away
                    continue;
                }
                double friction = p_.coulomb * std::copysign(1.0, m.vel != 0.0 ? m.vel : m.torque);
                double acc = (m.torque - p_.damping * m.vel - friction) / p_.inertia;
                m.vel += acc * dt;
                m.pos += m.vel * dt;
            }
            CanFrame fb;
            fb.id = canid::kFeedback + (i + 1);
            fb.dlc = 8;
            put<int32_t>(fb, 0, static_cast<int32_t>(std::lround(m.pos / (2 * M_PI) * p_.counts_per_rev)));
            put<float>(fb, 4, static_cast<float>(m.vel));
            bus_.send(fb);
        }
        timer.wait();
    }
}
