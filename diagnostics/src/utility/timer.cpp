#include "diagnostics/utility/timer.h"

// Constructor that initializes the vector of threads
Timer::Timer() {}

Timer::~Timer() {
    joinAll();
}

void Timer::start(std::function<void()> func, int timeout_ms) {
    threads_.emplace_back([func, timeout_ms]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        func();
    });
}

void Timer::joinAll() {
    for (std::thread &t : threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
}
