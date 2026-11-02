#include "rt_thread.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <thread>
#ifdef __linux__
#include <sys/mman.h>
#endif

namespace {
struct ThreadArgs {
    std::function<void()> fn;
};

void* trampoline(void* p) {
    auto* a = static_cast<ThreadArgs*>(p);
    a->fn();
    delete a;
    return nullptr;
}
}  // namespace

pthread_t start_thread(const char* name, std::function<void()> fn, int rt_priority) {
    pthread_t t{};
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (rt_priority > 0) {
        sched_param sp{};
        sp.sched_priority = rt_priority;
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        pthread_attr_setschedparam(&attr, &sp);
    }
    auto* args = new ThreadArgs{fn};
    int rc = pthread_create(&t, &attr, trampoline, args);
    if (rc != 0 && rt_priority > 0) {
        std::fprintf(stderr, "[%s] SCHED_FIFO unavailable (%s), using normal priority\n", name, std::strerror(rc));
        rc = pthread_create(&t, nullptr, trampoline, args);
    }
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        delete args;
        std::fprintf(stderr, "[%s] pthread_create failed: %s\n", name, std::strerror(rc));
        std::exit(1);
    }
#ifdef __linux__
    pthread_setname_np(t, name);
#endif
    return t;
}

int64_t now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

PeriodicTimer::PeriodicTimer(int64_t period_ns) : period_ns_(period_ns), next_ns_(now_ns() + period_ns) {}

int64_t PeriodicTimer::wait() {
#ifdef __linux__
    timespec ts{time_t(next_ns_ / 1'000'000'000), long(next_ns_ % 1'000'000'000)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
#else
    const int64_t d = next_ns_ - now_ns();
    if (d > 0) std::this_thread::sleep_for(std::chrono::nanoseconds(d));
#endif
    const int64_t late = now_ns() - next_ns_;
    next_ns_ += period_ns_;
    if (late > 10 * period_ns_) next_ns_ = now_ns() + period_ns_;  // overrun: resync instead of bursting
    return late > 0 ? late : 0;
}

void lock_memory() {
#ifdef __linux__
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        std::fprintf(stderr, "mlockall failed (run with CAP_IPC_LOCK for real-time use)\n");
#endif
}
