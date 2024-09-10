#include "../include/robot_state_machine/robot_state_machine.h"

RobotStateMachine::RobotStateMachine() {
    ros::NodeHandle nh;
    state_publisher = nh.advertise<std_msgs::String>("robot_state", 1, true);
}

void RobotStateMachine::publishState(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    state_publisher.publish(msg);
}
