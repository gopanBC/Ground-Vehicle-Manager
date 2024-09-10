#include "../include/robot_state_machine/robot_state_machine.h"
#include "../include/robot_state_machine/idle.h"
#include "../include/robot_state_machine/running.h"
#include "../include/robot_state_machine/error.h"

// Global state machine instance
RobotStateMachine* robot_state_machine;

// ROS callbacks
void goalCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received new goal: %s", msg->data.c_str());
    robot_state_machine->process_event(StartRunning());
}

void diagnosticsCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "ERROR") {
        robot_state_machine->process_event(ErrorDetected());
    } else if (msg->data == "CLEAR") {
        robot_state_machine->process_event(ErrorCleared());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_sm_node");
    ros::NodeHandle nh;
    // Initialize state machine
    RobotStateMachine state_machine;

    robot_state_machine = &state_machine;
    state_machine.initiate();

    // Set up ROS subscribers
    ros::Subscriber goal_sub = nh.subscribe("goal_topic", 10, goalCallback);
    ros::Subscriber diagnostics_sub = nh.subscribe("diagnostics", 10, diagnosticsCallback);

    ros::spin();
    return 0;
}
