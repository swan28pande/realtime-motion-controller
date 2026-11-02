// pthread helpers for the real-time loop.
#pragma once
#include <cstdint>
#include <functional>
#include <pthread.h>

// Starts a pthread. With rt_priority > 0 it asks for SCHED_FIFO and falls back
// to normal scheduling (with a warning) if the process lacks permission.
pthread_t start_thread(const char* name, std::function<void()> fn, int rt_priority = 0);

// Fixed-rate timer on absolute deadlines, so jitter does not accumulate.
class PeriodicTimer {
public:
    explicit PeriodicTimer(int64_t period_ns);
    // Sleeps until the next deadline. Returns how late the previous wake-up was (ns).
    int64_t wait();

private:
    int64_t period_ns_, next_ns_;
};

int64_t now_ns();
void lock_memory();  // mlockall on Linux, no-op elsewhere
