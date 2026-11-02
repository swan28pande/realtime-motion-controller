# Real-Time Motion Control System

A multi-threaded C++17 controller that runs a 1 kHz PID loop for simulated servo motors. It exchanges setpoints, torque commands, and encoder feedback over CAN bus (SocketCAN), and includes a C# operator console and MATLAB tuning scripts.

Stack: C++17, SocketCAN, pthreads, C#/.NET, MATLAB.

## Architecture

```
  C# Operator UI ──TCP 5555──► motion_controller ◄──── CAN bus ────► servo drives (servo_sim)
   (ui/OperatorUI)            ├─ pid_loop   1 kHz, SCHED_FIFO 80    0x200+n  torque command  ►
                              ├─ can_rx     encoder feedback        ◄ 0x180+n  encoder counts + velocity
                              ├─ operator   TCP bridge for the UI   ► 0x300+n  position setpoint (any node)
                              └─ main       status print + CSV log
```

| Thread | Role |
|---|---|
| `pid_loop` | Wakes on absolute deadlines (`clock_nanosleep`, `TIMER_ABSTIME`). Runs one PID per axis and sends torque frames. Tracks worst-case wake-up latency. |
| `can_rx` | Decodes encoder feedback and setpoint frames into lock-free per-axis state (`std::atomic`). |
| `servo_sim` | Motor physics `J·ω' = τ − bω − friction` (with stiction) at 10 kHz. Publishes 4096-count encoder feedback at 1 kHz. |
| `operator` | TCP bridge: receives `SET <axis> <deg>`, streams status at 20 Hz. |

**PID** (`src/pid.hpp`): derivative on measurement (no kick on setpoint steps), filtered derivative, torque clamp, and conditional-integration anti-windup.

**Safety:** if an axis gets no encoder feedback for 50 ms, its integrator resets and it is commanded to zero torque.

**CAN backends** (`src/can_bus.cpp`): raw SocketCAN on Linux, or an in-process `loopback` bus so the whole system also runs without CAN hardware.

## Build and run

```bash
cmake -S . -B build && cmake --build build -j

# Everything in one process on the loopback bus, stepping through setpoints
./build/motion_controller --demo --log run.csv
```

**Over SocketCAN** (Linux, virtual CAN):

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
./build/servo_sim --bus vcan0 &
sudo ./build/motion_controller --bus vcan0      # sudo for SCHED_FIFO + mlockall
candump vcan0                                    # watch the traffic
cansend vcan0 301#0000B442                       # axis 1 -> 90.0 deg (little-endian float)
```

**Operator UI** (.NET 10):

```bash
cd ui/OperatorUI
dotnet run                         # live view; type "1 90", "all 0", "q"
dotnet run -- 127.0.0.1 5555 --set 2 -30
```

```
REAL-TIME MOTION CONTROL - OPERATOR
control cycles       7870   max loop latency    ... us

axis   setpoint   position    error  vel rad/s  torque Nm
   1      90.00      90.18    -0.18      0.000     -0.031
   2     -30.00     -30.50     0.50      0.000      0.048
```

**MATLAB** (also runs in GNU Octave):

- `matlab/pid_tuning.m` models one axis with the same plant and discrete PID, so you can tune gains offline. With the defaults (Kp 20, Ki 10, Kd 1.2) it shows about 1° overshoot on a 90° step.
- `plot_log('run.csv')` plots tracking from a `--log` recording.

## Options

`motion_controller [--bus loopback|vcan0] [--axes 2] [--rate 1000] [--port 5555] [--listen 127.0.0.1] [--sim] [--demo] [--log run.csv] [--duration SEC]`

`--sim` runs the simulated drives in-process; it is implied by `--bus loopback`.

## Status

Early skeleton:
- Tested on Linux in Docker over the loopback bus, with the C# UI driving it.
- The SocketCAN backend builds on Linux but has not yet been run against a real `vcan0` interface.
- Loop latency depends heavily on the host. For real-time numbers, run on a PREEMPT_RT kernel with `sudo`, not in a container.
