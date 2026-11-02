// motion_controller: real-time PID controller for CAN servo drives.
//
//   motion_controller [--bus loopback|vcan0] [--axes 2] [--rate 1000] [--port 5555] [--listen 127.0.0.1]
//                     [--sim] [--log run.csv] [--demo] [--duration SEC]
//
// --bus loopback implies --sim (the simulated drives run in this process).
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "can_bus.hpp"
#include "controller.hpp"
#include "operator_server.hpp"
#include "rt_thread.hpp"
#include "servo_sim.hpp"

static std::atomic<bool> g_running{true};

int main(int argc, char** argv) {
    std::string bus_name = "loopback", log_path, listen_addr = "127.0.0.1";
    int axes = 2, rate = 1000, port = 5555;
    double duration = 0;
    bool sim = false, demo = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&] { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        if (a == "--bus") bus_name = next();
        else if (a == "--axes") axes = std::atoi(next().c_str());
        else if (a == "--rate") rate = std::atoi(next().c_str());
        else if (a == "--port") port = std::atoi(next().c_str());
        else if (a == "--log") log_path = next();
        else if (a == "--listen") listen_addr = next();
        else if (a == "--duration") duration = std::atof(next().c_str());
        else if (a == "--sim") sim = true;
        else if (a == "--demo") demo = true;
        else { std::fprintf(stderr, "unknown option %s (see header of src/main.cpp)\n", a.c_str()); return 1; }
    }
    if (bus_name == "loopback") sim = true;

    std::signal(SIGINT, [](int) { g_running = false; });
    std::signal(SIGTERM, [](int) { g_running = false; });
    std::signal(SIGPIPE, SIG_IGN);  // a closed UI socket must not kill the controller
    lock_memory();

    std::unique_ptr<CanBus> ctl_bus, sim_bus;
    try {
        ctl_bus = open_bus(bus_name);
        if (sim) sim_bus = open_bus(bus_name);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }

    Controller ctl(*ctl_bus, axes, rate);
    ServoSim* servo = sim ? new ServoSim(*sim_bus, axes, rate) : nullptr;

    std::printf("motion_controller: %d axes, %d Hz, bus=%s%s\n", axes, rate, bus_name.c_str(),
                sim ? " (+ simulated drives)" : "");

    pthread_t threads[5];
    int nt = 0;
    threads[nt++] = start_thread("can_rx", [&] { ctl.rx_loop(g_running); }, 70);
    threads[nt++] = start_thread("pid_loop", [&] { ctl.control_loop(g_running); }, 80);
    if (servo) threads[nt++] = start_thread("servo_sim", [&] { servo->run(g_running); }, 60);
    threads[nt++] = start_thread("operator", [&] { run_operator_server(ctl, listen_addr, port, g_running); });

    // Demo: step every axis through a few setpoints.
    if (demo) {
        threads[nt++] = start_thread("demo", [&] {
            const double steps[] = {90, -45, 180, 0};
            for (int k = 0; g_running; k = (k + 1) % 4) {
                for (int ax = 1; ax <= axes; ++ax) ctl.set_setpoint_deg(ax, steps[k] / ax);
                for (int t = 0; t < 150 && g_running; ++t) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    // Main thread: status print (1 Hz) and CSV log (100 Hz).
    FILE* log = log_path.empty() ? nullptr : std::fopen(log_path.c_str(), "w");
    if (log) {
        std::fprintf(log, "t");
        for (int i = 1; i <= axes; ++i) std::fprintf(log, ",sp%d,pos%d,effort%d", i, i, i);
        std::fprintf(log, "\n");
    }
    const int64_t t0 = now_ns();
    for (int tick = 0; g_running; ++tick) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        const double t = (now_ns() - t0) * 1e-9;
        if (log) {
            std::fprintf(log, "%.3f", t);
            for (int i = 0; i < axes; ++i) {
                const AxisState& a = ctl.axis(i);
                std::fprintf(log, ",%.3f,%.3f,%.4f", a.setpoint * 180 / M_PI, a.position * 180 / M_PI, a.effort.load());
            }
            std::fprintf(log, "\n");
        }
        if (tick % 100 == 0) {
            std::printf("t=%5.1fs cycles=%lld max_late=%lldus |", t, (long long)ctl.stats().cycles.load(),
                        (long long)(ctl.stats().max_late_ns / 1000));
            for (int i = 0; i < axes; ++i) {
                const AxisState& a = ctl.axis(i);
                std::printf(" ax%d sp=%7.2f pos=%7.2f u=%+5.2f |", i + 1, a.setpoint * 180 / M_PI,
                            a.position * 180 / M_PI, a.effort.load());
            }
            std::printf("\n");
            std::fflush(stdout);
        }
        if (duration > 0 && t >= duration) g_running = false;
    }

    for (int i = 0; i < nt; ++i) pthread_join(threads[i], nullptr);
    if (log) std::fclose(log);
    delete servo;
    std::printf("stopped after %lld control cycles\n", (long long)ctl.stats().cycles.load());
    return 0;
}
