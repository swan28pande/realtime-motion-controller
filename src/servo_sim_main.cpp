// servo_sim: simulated CAN servo drives as a separate process (for SocketCAN / vcan0).
//
//   servo_sim --bus vcan0 [--axes 2] [--rate 1000]
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "can_bus.hpp"
#include "servo_sim.hpp"

static std::atomic<bool> g_running{true};

int main(int argc, char** argv) {
    std::string bus_name = "vcan0";
    int axes = 2, rate = 1000;
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string a = argv[i];
        if (a == "--bus") bus_name = argv[i + 1];
        else if (a == "--axes") axes = std::atoi(argv[i + 1]);
        else if (a == "--rate") rate = std::atoi(argv[i + 1]);
    }
    std::signal(SIGINT, [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });
    try {
        auto bus = open_bus(bus_name);
        std::printf("servo_sim: %d drives on %s at %d Hz\n", axes, bus_name.c_str(), rate);
        ServoSim(*bus, axes, rate).run(g_running);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    return 0;
}
