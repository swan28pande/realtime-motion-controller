#include "operator_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

#include "rt_thread.hpp"

namespace {
std::string status_line(const Controller& ctl) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    os << "STATUS " << ctl.stats().cycles << ' ' << ctl.stats().max_late_ns / 1000;
    for (int i = 0; i < ctl.num_axes(); ++i) {
        const AxisState& a = ctl.axis(i);
        os << ' ' << (i + 1) << ':' << a.setpoint * 180 / M_PI << ':' << a.position * 180 / M_PI << ':'
           << a.velocity.load() << ':' << a.effort.load();
    }
    os << '\n';
    return os.str();
}

void handle_line(Controller& ctl, const std::string& line) {
    std::istringstream is(line);
    std::string cmd;
    int axis;
    double deg;
    if (is >> cmd >> axis >> deg && cmd == "SET") ctl.set_setpoint_deg(axis, deg);
}
}  // namespace

void run_operator_server(Controller& ctl, const std::string& listen_addr, int port, const std::atomic<bool>& running) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, listen_addr.c_str(), &addr.sin_addr) != 1) {
        std::fprintf(stderr, "bad listen address %s\n", listen_addr.c_str());
        close(srv);
        return;
    }
    if (bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || listen(srv, 1) < 0) {
        std::perror("operator server");
        close(srv);
        return;
    }
    std::printf("Operator UI server on %s:%d\n", listen_addr.c_str(), port);

    int client = -1;
    std::string buf;
    int64_t next_status = now_ns();
    while (running) {
        pollfd p{client >= 0 ? client : srv, POLLIN, 0};
        if (poll(&p, 1, 10) > 0) {
            if (client < 0) {
                client = accept(srv, nullptr, nullptr);
                buf.clear();
            } else {
                char tmp[256];
                ssize_t n = recv(client, tmp, sizeof(tmp), 0);
                if (n <= 0) {
                    close(client);
                    client = -1;
                    continue;
                }
                buf.append(tmp, n);
                for (size_t nl; (nl = buf.find('\n')) != std::string::npos; buf.erase(0, nl + 1))
                    handle_line(ctl, buf.substr(0, nl));
            }
        }
        if (client >= 0 && now_ns() >= next_status) {
            next_status = now_ns() + 50'000'000;
            std::string s = status_line(ctl);
            if (send(client, s.data(), s.size(), 0) < 0) {
                close(client);
                client = -1;
            }
        }
    }
    if (client >= 0) close(client);
    close(srv);
}
