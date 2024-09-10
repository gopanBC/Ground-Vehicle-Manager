#ifndef ROS_MIDDLEWARE_H
#define ROS_MIDDLEWARE_H

#include "middleware_interface.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

class ROSMiddleware : public MiddlewareInterface {
public:
    ROSMiddleware(const std::string& topic);

    void initialize(int argc, char** argv) override;
    void spin() override;
    void publish(const std::string& message) override;
    void subscribeAll() override;

private:
    std::string topic_;
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Publisher publisher_;
    std::vector<ros::Subscriber> subscriber_list_;
};

#endif // ROS_MIDDLEWARE_H
