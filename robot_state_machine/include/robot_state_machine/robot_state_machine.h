#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/event.hpp>

#include <memory>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <realtime_tools/realtime_publisher.h>

// Forward declarations for states
struct Idle;


// Define events
struct StartRunning : public boost::statechart::event<StartRunning> {};
struct ErrorDetected : public boost::statechart::event<ErrorDetected> {};
struct ErrorCleared : public boost::statechart::event<ErrorCleared> {};
struct GoalReached : public boost::statechart::event<GoalReached> {};
struct RoutePlanned : public boost::statechart::event<RoutePlanned> {};
struct Stop : public boost::statechart::event<Stop> {};

// State machine definition
struct RobotStateMachine : public boost::statechart::state_machine<RobotStateMachine, Idle> {
    RobotStateMachine();
    void publishState(const std::string& state);
    
private:
    ros::Publisher state_publisher;
};

#endif // ROBOT_STATE_MACHINE_H
