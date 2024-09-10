#include "../include/robot_state_machine/idle.h"
#include "../include/robot_state_machine/running.h"
#include "../include/robot_state_machine/error.h"

Idle::Idle(my_context ctx) : my_base(ctx) {
    ROS_INFO("Entering IDLE state.");
    this->context<RobotStateMachine>().publishState("IDLE");
}
 boost::statechart::result Idle::react(const StartRunning& running) {
    return transit<Running>();
 }

 boost::statechart::result Idle::react(const ErrorDetected& error) {
    return transit<Error>(); 
 }
 
Idle::~Idle() {}