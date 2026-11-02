// Simulated servo drives: each motor is J*dw/dt = torque - b*w (+ Coulomb friction).
// They take torque commands from CAN and publish encoder feedback on CAN.
#pragma once
#include <atomic>
#include <vector>

#include "can_bus.hpp"

struct MotorParams {
    double inertia = 0.01;          // kg m^2
    double damping = 0.1;           // N m s/rad
    double coulomb = 0.05;          // N m
    int counts_per_rev = 4096;      // encoder resolution
};

class ServoSim {
public:
    ServoSim(CanBus& bus, int num_motors, int feedback_hz = 1000, MotorParams p = {});
    void run(const std::atomic<bool>& running);  // physics + CAN loop, call from its own thread

private:
    struct Motor {
        double pos = 0, vel = 0, torque = 0;
    };
    CanBus& bus_;
    std::vector<Motor> motors_;
    int feedback_hz_;
    MotorParams p_;
};
