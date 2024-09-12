#include "../../include/robot_state_machine/controllers/controller_factory.h"
#include "../../include/robot_state_machine/controllers/pure_pursuit.h"

ControllerFactory& ControllerFactory::instance() {
    static ControllerFactory instance;
    return instance;
}

void ControllerFactory::registerController(const std::string& type, CreatorFunc creator) {
    creators_[type] = creator;
}

std::unique_ptr<ControllerInterface> ControllerFactory::createController(const std::string& type, ros::NodeHandle& nh) const {
    auto it = creators_.find(type);
    if (it != creators_.end()) {
        return it->second(nh);
    }
    return nullptr;
}

// Register controllers
namespace {
    bool registered = []() {
        ControllerFactory::instance().registerController("PPC", [](ros::NodeHandle& nh) {
            return std::make_unique<PurePursuit>(nh);
        });
        return true;
    }();
}
