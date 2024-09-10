#include "diagnostics/rti/real_time_scheduler.h"

RealTimeScheduler::RealTimeScheduler(std::function<void()> task, std::chrono::milliseconds interval)
    : task_(task), interval_(interval), stop_(false) {
    thread_ = std::thread(&RealTimeScheduler::run, this);
}

RealTimeScheduler::~RealTimeScheduler() {
    stop_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void RealTimeScheduler::run() {
    while (!stop_) {
        auto start_time = std::chrono::steady_clock::now();
        task_();
        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::this_thread::sleep_for(interval_ - elapsed);
    }
}

