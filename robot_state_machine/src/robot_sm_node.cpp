#include "../include/robot_state_machine/robot_state_machine.h"
#include "../include/robot_state_machine/idle.h"
#include "../include/robot_state_machine/running.h"
#include "../include/robot_state_machine/error.h"
#include "../include/robot_state_machine/utilities/waypoint_reader.h"

#include "../msg/diagnostics.pb.h"

#include <zmq.hpp>

#include <geometry_msgs/Pose.h>

// Global state machine instance
RobotStateMachine* robot_state_machine;

zmq::context_t context(1);
zmq::socket_t subscriber(context, ZMQ_SUB);

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
    DiagnosticsMessage diag_msg;
    diag_msg.ParseFromString(msg->data); //deserialise protobuf string

    if (diag_msg.status() == DiagnosticsMessage::ERROR) {
        robot_state_machine->process_event(ErrorDetected());
    } else if (diag_msg.status() == DiagnosticsMessage::OK ) {
        robot_state_machine->process_event(ErrorCleared());
    }
}

void diagnosticsCallback(const std::string& msg) {
    DiagnosticsMessage diag_msg;
    diag_msg.ParseFromString(msg); //deserialise protobuf string

    if (diag_msg.status() == DiagnosticsMessage::ERROR) {
        robot_state_machine->process_event(ErrorDetected());
    } else if (diag_msg.status() == DiagnosticsMessage::OK ) {
        robot_state_machine->process_event(ErrorCleared());
    }
}

void initZeroMQ() {
    // Connect to the publisher (same address used for the publisher)
    subscriber.connect("tcp://localhost:5556");

    // Subscribe to all topics (empty subscription means all messages)
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    std::cout << "Subscribed to ZeroMQ messages on tcp://localhost:5556" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_sm_node");
    ros::NodeHandle nh;
    // Initialize state machine
    RobotStateMachine state_machine;

    robot_state_machine = &state_machine;
    state_machine.initiate();
    initZeroMQ();

    //update path using rospkg find
    WPReader wp_reader("/home/anthar/ros_space/src/ground_vehicle_assignment/robot_state_machine/waypoints/wps.csv");
    sendRoutes(&wp_reader);

    // Set up ROS subscribers
    ros::Subscriber goal_sub = nh.subscribe("goal_topic", 10, goalCallback);
    //ros::Subscriber diagnostics_sub = nh.subscribe("diagnostics", 10, diagnosticsCallback);

    ros::Rate rate(100);
    while (true) {
        zmq::message_t message;
        subscriber.recv(&message);
        std::string message_str(static_cast<char*>(message.data()), message.size());
        diagnosticsCallback(message_str);
        //std::cout << "Received message: " << message_str << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
