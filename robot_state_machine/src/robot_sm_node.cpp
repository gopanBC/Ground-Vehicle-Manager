#include "../include/robot_state_machine/robot_state_machine.h"
#include "../include/robot_state_machine/idle.h"
#include "../include/robot_state_machine/running.h"
#include "../include/robot_state_machine/error.h"
#include "../include/robot_state_machine/utilities/waypoint_reader.h"

#include <geometry_msgs/Pose.h>

// Global state machine instance
RobotStateMachine* robot_state_machine;

void signalHandler(int signum) {
    robot_state_machine->process_event(Stop());
}

void sendRoutes(WPReader* wp_reader) {
    wp_reader->readWaypoints();
    auto path_points_x = wp_reader->getPathPointsX();
    auto path_points_y = wp_reader->getPathPointsY();
    auto path_points_yaw = wp_reader->getPathPointsYaw();
    Routes routes(path_points_x, path_points_y, path_points_yaw);
    robot_state_machine->process_event(routes);
}

// ROS callbacks
void goalCallback(const geometry_msgs::Pose& msg) {
    //ROS_INFO("Received new goal: %s", msg->data.c_str());
    robot_state_machine->process_event(StartRunning(msg));
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

    //update path using rospkg find
    WPReader wp_reader("/home/anthar/ros_space/src/ground_vehicle_assignment/robot_state_machine/waypoints/wps.csv");
    sendRoutes(&wp_reader);

    // Set up ROS subscribers
    ros::Subscriber goal_sub = nh.subscribe("goal_topic", 10, goalCallback);
    ros::Subscriber diagnostics_sub = nh.subscribe("diagnostics", 10, diagnosticsCallback);

    ros::spin();
    return 0;
}
