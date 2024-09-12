#include "../include/robot_state_machine/move.h"
#include "../include/robot_state_machine/error.h"
#include "../include/robot_state_machine/controllers/controller_factory.h"

#include <thread>
#include <chrono>

#include <ros/ros.h>

Control::Control(my_context ctx) : my_base(ctx), 
                running_(true) {
    //behaviour_plan_(this->context<Move>().getBehaviourPlan())          
    this->context<RobotStateMachine>().publishState("MOVE:CONTROL");
    this->context<RobotStateMachine>().getRoutes(x_, y_, yaw_);
    control_thread_ = std::thread(&Control::run, this);
}
//behaviour_plan_mutex_(this->context<Move>().getBehaviourPlanMutex())

boost::statechart::result Control::react(const ErrorDetected& error) {
    return transit<Error>(); 
}

boost::statechart::result Control::react(const Stop& stop) {
    running_ = true;
    return discard_event(); 
}

void Control::run() {
    using namespace std::chrono_literals;
    auto nh = ros::NodeHandle();
    auto controller = ControllerFactory::instance().createController("PPC", nh);
    if (controller) {
        controller->start(x_, y_, yaw_);
    }
    else {
        //ROS_ERROR("Failed to create controller of type: %s", controller_type.c_str());
        return;
    }

    while (running_) {
        auto vehicle_state = this->context<RobotStateMachine>().getVehicleState();
        controller->update(vehicle_state);
        std::this_thread::sleep_for(std::chrono::milliseconds(100ms));
    }
}

void Control::stop() {
    running_ = false;
}

Control::~Control() {
    stop();
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}