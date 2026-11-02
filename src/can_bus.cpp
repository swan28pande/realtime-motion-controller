#include "can_bus.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <vector>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

// Every endpoint on the loopback bus sees frames sent by the others, like a real bus
// (SocketCAN also does not echo a socket's own frames by default).
class LoopbackHub {
public:
    struct Endpoint {
        std::mutex m;
        std::condition_variable cv;
        std::deque<CanFrame> q;
    };

    static LoopbackHub& instance() { static LoopbackHub hub; return hub; }

    std::shared_ptr<Endpoint> attach() {
        auto ep = std::make_shared<Endpoint>();
        std::lock_guard<std::mutex> lk(m_);
        eps_.push_back(ep);
        return ep;
    }

    void detach(const std::shared_ptr<Endpoint>& ep) {
        std::lock_guard<std::mutex> lk(m_);
        eps_.erase(std::remove(eps_.begin(), eps_.end(), ep), eps_.end());
    }

    void broadcast(const Endpoint* from, const CanFrame& f) {
        std::lock_guard<std::mutex> lk(m_);
        for (auto& ep : eps_) {
            if (ep.get() == from) continue;
            {
                std::lock_guard<std::mutex> elk(ep->m);
                if (ep->q.size() > 1024) ep->q.pop_front();  // drop oldest, like a full rx buffer
                ep->q.push_back(f);
            }
            ep->cv.notify_one();
        }
    }

private:
    std::mutex m_;
    std::vector<std::shared_ptr<Endpoint>> eps_;
};

class LoopbackBus : public CanBus {
public:
    LoopbackBus() : ep_(LoopbackHub::instance().attach()) {}
    ~LoopbackBus() override { LoopbackHub::instance().detach(ep_); }

    bool send(const CanFrame& f) override {
        LoopbackHub::instance().broadcast(ep_.get(), f);
        return true;
    }

    bool recv(CanFrame& f, int timeout_ms) override {
        std::unique_lock<std::mutex> lk(ep_->m);
        if (!ep_->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [&] { return !ep_->q.empty(); }))
            return false;
        f = ep_->q.front();
        ep_->q.pop_front();
        return true;
    }

private:
    std::shared_ptr<LoopbackHub::Endpoint> ep_;
};

#ifdef __linux__
class SocketCanBus : public CanBus {
public:
    explicit SocketCanBus(const std::string& ifname) {
        fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd_ < 0) throw std::runtime_error("socket(PF_CAN) failed");
        ifreq ifr{};
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
            close(fd_);
            throw std::runtime_error("CAN interface not found: " + ifname);
        }
        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(fd_);
            throw std::runtime_error("bind failed on " + ifname);
        }
    }
    ~SocketCanBus() override { close(fd_); }

    bool send(const CanFrame& f) override {
        can_frame cf{};
        cf.can_id = f.id;
        cf.can_dlc = f.dlc;
        std::memcpy(cf.data, f.data, 8);
        return write(fd_, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf));
    }

    bool recv(CanFrame& f, int timeout_ms) override {
        pollfd p{fd_, POLLIN, 0};
        if (poll(&p, 1, timeout_ms) <= 0) return false;
        can_frame cf{};
        if (read(fd_, &cf, sizeof(cf)) != static_cast<ssize_t>(sizeof(cf))) return false;
        f.id = cf.can_id & CAN_SFF_MASK;
        f.dlc = cf.can_dlc;
        std::memcpy(f.data, cf.data, 8);
        return true;
    }

private:
    int fd_ = -1;
};
#endif

}  // namespace

std::unique_ptr<CanBus> open_bus(const std::string& name) {
    if (name == "loopback") return std::make_unique<LoopbackBus>();
#ifdef __linux__
    return std::make_unique<SocketCanBus>(name);
#else
    throw std::runtime_error("SocketCAN needs Linux; use --bus loopback on this platform");
#endif
}
