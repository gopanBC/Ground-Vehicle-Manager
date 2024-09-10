#ifndef ROS2_MIDDLEWARE_H
#define ROS2_MIDDLEWARE_H

#include "middleware_interface.h"

class ROS2Middleware : public MiddlewareInterface {
public:
    ROS2Middleware(const std::string& topic);

    void initialize(int argc, char** argv) override;
    void spin() override;
    void publish(const std::string& message) override;
    void subscribeAll() override;

private:
    std::string topic_;
    //add vector to hold all subscribers.
};

#endif // ROS2_MIDDLEWARE_H

