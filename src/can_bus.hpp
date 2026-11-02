// CAN bus abstraction: SocketCAN on Linux, plus an in-process loopback bus
// so the whole system also runs on machines without CAN hardware.
#pragma once
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

struct CanFrame {
    uint32_t id = 0;
    uint8_t dlc = 0;
    uint8_t data[8] = {};
};

// Message IDs (11-bit). Node id is added to the base.
namespace canid {
constexpr uint32_t kCommand = 0x200;   // controller -> drive: float torque [Nm]
constexpr uint32_t kFeedback = 0x180;  // drive -> controller: int32 encoder counts, float velocity [rad/s]
constexpr uint32_t kSetpoint = 0x300;  // anyone -> controller: float position [deg]
}  // namespace canid

template <typename T>
inline void put(CanFrame& f, int offset, T v) { std::memcpy(f.data + offset, &v, sizeof(T)); }
template <typename T>
inline T get(const CanFrame& f, int offset) { T v; std::memcpy(&v, f.data + offset, sizeof(T)); return v; }

class CanBus {
public:
    virtual ~CanBus() = default;
    virtual bool send(const CanFrame& f) = 0;
    // Waits up to timeout_ms for a frame. Returns false on timeout.
    virtual bool recv(CanFrame& f, int timeout_ms) = 0;
};

// "loopback" -> shared in-process bus, anything else -> SocketCAN interface name (e.g. vcan0).
std::unique_ptr<CanBus> open_bus(const std::string& name);
