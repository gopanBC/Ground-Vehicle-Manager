#ifndef CONTROLLER_FACTORY_H
#define CONTROLLER_FACTORY_H

#include "controller_interface.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

#include <ros/ros.h>

class ControllerFactory {
public:
    using CreatorFunc = std::function<std::unique_ptr<ControllerInterface>(ros::NodeHandle&)>;

    static ControllerFactory& instance();

    void registerController(const std::string& type, CreatorFunc creator);

    std::unique_ptr<ControllerInterface> createController(const std::string& type, ros::NodeHandle& nh) const;

private:
    ControllerFactory() = default;

    std::unordered_map<std::string, CreatorFunc> creators_;
};

#endif // CONTROLLER_FACTORY_H
