#include "../include/robot_state_machine/running.h"
#include "../include/robot_state_machine/error.h"

Running::Running(my_context ctx) : my_base(ctx) {
    ROS_INFO("Entering RUNNING state.");
    this->context<RobotStateMachine>().publishState("RUNNING");
}

boost::statechart::result Running::react(const ErrorDetected& error) {
    return transit<Error>(); 
}