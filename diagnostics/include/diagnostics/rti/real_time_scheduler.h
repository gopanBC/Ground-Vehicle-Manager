#ifndef REAL_TIME_SCHEDULER_H
#define REAL_TIME_SCHEDULER_H

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>

class RealTimeScheduler {
public:
    RealTimeScheduler(std::function<void()> task, std::chrono::nanoseconds interval, int priority);
    ~RealTimeScheduler();

private:
    void run();
     bool setRealTimePriority(std::thread& thread, int priority);

    std::function<void()> task_;
    std::chrono::nanoseconds interval_;
    std::thread thread_;
    std::atomic<bool> stop_;
};

#endif // REAL_TIME_SCHEDULER_H

