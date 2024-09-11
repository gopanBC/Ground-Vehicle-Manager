#include "../include/robot_state_machine/move.h"
#include "../include/robot_state_machine/error.h"

#include <thread>
#include <chrono>

Control::Control(my_context ctx) : my_base(ctx), 
                running_(true),
                behaviour_plan_(this->context<Move>().getBehaviourPlan()) {
    this->context<RobotStateMachine>().publishState("MOVE:CONTROL");
    control_thread_ = std::thread(&Control::run, this);
}
//behaviour_plan_mutex_(this->context<Move>().getBehaviourPlanMutex())

boost::statechart::result Control::react(const ErrorDetected& error) {
    return transit<Error>(); 
}

boost::statechart::result Control::react(const Stop& stop) {
    running_ = true;
    return discard_event(); 
}

void Control::run() {
    using namespace std::chrono_literals;
    extern std::atomic<bool> ctrl_flag;
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100ms));
        if(!behaviour_plan_.empty()) {
            //execute control commands here.
        }   
    }
}

void Control::stop() {
    running_ = false;
}

Control::~Control() {
    stop();
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}