#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <thread>
#include <functional>
#include <chrono>
#include <vector>

class Timer {
public:
    // Constructor to initialize with an empty thread list
    Timer();
    ~Timer();

    // Method to start a delayed function call with a specified timeout (ms) and callback
    void start(std::function<void()> func, int timeout_ms);

    // Method to join all threads in the vector
    void joinAll();

private:
    std::vector<std::thread> threads_; // Vector to hold all the threads
};

#endif // TIMER_H
