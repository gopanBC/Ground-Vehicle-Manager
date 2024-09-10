#include "../include/robot_state_machine/error.h"
#include "../include/robot_state_machine/idle.h"

Error::Error(my_context ctx) : my_base(ctx) {
    ROS_ERROR("Entering ERROR state.");
    this->context<RobotStateMachine>().publishState("ERROR");
}

boost::statechart::result Error::react(const ErrorCleared& err_cleared) {
    return transit<Idle>();
}

Error::~Error() {
    ROS_INFO("Error state cleared, transitioning back to the previous state.");
}
