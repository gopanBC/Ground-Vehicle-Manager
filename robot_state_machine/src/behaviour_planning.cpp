// #include "../include/robot_state_machine/behaviour_planning.h"
#include "../include/robot_state_machine/move.h"
#include "../include/robot_state_machine/error.h"
#include "../include/robot_state_machine/idle.h"

#include <thread>
#include <chrono>

BehaviourPlanning::BehaviourPlanning(my_context ctx) : my_base(ctx), running_(true),
                                    behaviour_plan_(this->context<Move>().getBehaviourPlan()) {
    this->context<RobotStateMachine>().publishState("MOVE:BP");
    planning_thread_ = std::thread(&BehaviourPlanning::run, this);
}

boost::statechart::result BehaviourPlanning::react(const GoalReached& goal_reached) {
    return transit<Idle>(); 
}

boost::statechart::result BehaviourPlanning::react(const ErrorDetected& error) {
    return transit<Error>(); 
}

boost::statechart::result BehaviourPlanning::react(const Stop& stop) {
    running_ = true;
    return discard_event(); 
}

void BehaviourPlanning::run()
{
    using namespace std::chrono_literals;
    while (running_) {
        //check goal reached condition 
        //post_event(GoalReached())
        std::this_thread::sleep_for(100ms);
    }
}

void BehaviourPlanning::stop()
{
     running_ = false;
}

BehaviourPlanning::~BehaviourPlanning() {
    stop();
    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
}