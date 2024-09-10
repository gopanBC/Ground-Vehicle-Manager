#ifndef REAL_TIME_SCHEDULER_H
#define REAL_TIME_SCHEDULER_H

#include <thread>
#include <chrono>
#include <functional>

class RealTimeScheduler {
public:
    RealTimeScheduler(std::function<void()> task, std::chrono::milliseconds interval);
    ~RealTimeScheduler();

private:
    void run();

    std::function<void()> task_;
    std::chrono::milliseconds interval_;
    std::thread thread_;
    bool stop_;
};

#endif // REAL_TIME_SCHEDULER_H

