#include "diagnostics/middleware/ros_middleware.h"

ROSMiddleware::ROSMiddleware(const std::string& topic) : topic_(topic) {}

void ROSMiddleware::initialize(int argc, char** argv) {
    ros::init(argc, argv, "sensor_monitor_node");
    nh_ = std::make_unique<ros::NodeHandle>();
    publisher_ = nh_->advertise<std_msgs::String>(topic_, 10);
}

void ROSMiddleware::spin() {
    ros::spin();
}

void ROSMiddleware::publish(const std::string& message) {
    std_msgs::String msg;
    msg.data = message;
    publisher_.publish(msg);
}

void ROSMiddleware::subscribeAll() {
    for(auto topics_and_cb : subscription_list) {
        //message_callback_ = callback;  // Store the callback function

        auto subscriber = nh_->subscribe<std_msgs::String>(topics_and_cb.first, 10, [&](const std_msgs::String::ConstPtr& msg) {
            topics_and_cb.second((msg->data));  // Call the callback with the message data
        });
        subscriber_list_.push_back(subscriber);
    }
}

