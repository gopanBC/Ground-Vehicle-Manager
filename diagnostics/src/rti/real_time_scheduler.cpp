#include "diagnostics/rti/real_time_scheduler.h"

#include <sched.h>

#include <ros/ros.h>

RealTimeScheduler::RealTimeScheduler(std::function<void()> task, std::chrono::nanoseconds interval, int priority)
    : task_(task), interval_(interval), stop_(false) {
    thread_ = std::thread(&RealTimeScheduler::run, this);
    setRealTimePriority(thread_, priority);
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

bool RealTimeScheduler::setRealTimePriority(std::thread& thread, int priority) {
    struct sched_param param;
    param.sched_priority = priority; // Set to a real-time priority level (higher than most processes)

    pthread_t pthread_id = thread.native_handle();
    if (pthread_setschedparam(pthread_id, SCHED_FIFO, &param) != 0) {
        ROS_WARN("Failed to set real-time priority for control execution thread.");
        return false;
    }
    return true;
}