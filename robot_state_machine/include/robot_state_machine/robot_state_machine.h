#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include "utilities/data/vehicle_state.h"

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/event.hpp>

#include <memory>
#include <string>
#include <atomic>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>

#include <realtime_tools/realtime_publisher.h>

// Forward declarations for states
struct Idle;


// Define events
struct StartRunning : public boost::statechart::event<StartRunning> 
{
    StartRunning(const geometry_msgs::Pose& goal) : goal(goal) {}
    geometry_msgs::Pose goal;
};
struct ErrorDetected : public boost::statechart::event<ErrorDetected> {};
struct ErrorCleared : public boost::statechart::event<ErrorCleared> {};
struct GoalReached : public boost::statechart::event<GoalReached> {};
struct RoutePlanned : public boost::statechart::event<RoutePlanned> {};
struct Stop : public boost::statechart::event<Stop> {};
struct Routes : public boost::statechart::event<Routes>
{
    Routes(const std::vector<double>& x, std::vector<double>& y, std::vector<double>& yaw) :
        x(x), y(y), yaw(yaw) {}
    std::vector<double> x, y, yaw;
};

// State machine definition
struct RobotStateMachine : public boost::statechart::state_machine<RobotStateMachine, Idle> {
    RobotStateMachine();
    void publishState(const std::string& state);
    void saveRoutes(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& yaw);
    void getRoutes(std::vector<double>& x, std::vector<double>& y, std::vector<double>& yaw);
    VehicleState getVehicleState();
    void setVehicleState(const VehicleState& new_state);
    void vehicleStateCB(const gazebo_msgs::ModelStates::ConstPtr& state_info);
    void setNewGoal(const geometry_msgs::Pose& new_goal);
    geometry_msgs::Pose& getGoal();

private:
    ros::Publisher state_publisher;
    ros::Subscriber state_subscriber;
    std::vector<double> x_, y_, yaw_;
    std::atomic<VehicleState> atomic_state;
    geometry_msgs::Pose goal_;
};

#endif // ROBOT_STATE_MACHINE_H
