// #include "../include/robot_state_machine/behaviour_planning.h"
#include "../include/robot_state_machine/move.h"
#include "../include/robot_state_machine/error.h"
#include "../include/robot_state_machine/idle.h"

#include <thread>
#include <chrono>

BehaviourPlanning::BehaviourPlanning(my_context ctx) : my_base(ctx), running_(true),
                                    behaviour_plan_(this->context<Move>().getBehaviourPlan()) {
    this->context<RobotStateMachine>().publishState("MOVE:BP");
    goal_ = this->context<RobotStateMachine>().getGoal();
    planning_thread_ = std::thread(&BehaviourPlanning::run, this);
}

boost::statechart::result BehaviourPlanning::react(const GoalReached& goal_reached) {
    return transit<Idle>(); 
}

boost::statechart::result BehaviourPlanning::react(const ErrorDetected& error) {
    return transit<Error>(); 
}

boost::statechart::result BehaviourPlanning::react(const Stop& stop) {
    running_ = true;
    return discard_event(); 
}

double BehaviourPlanning::distance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
    double dx = pose2.position.x - pose1.position.x;
    double dy = pose2.position.y - pose1.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::Pose BehaviourPlanning::convertVehicleState(const VehicleState& vehicle_state) {
    geometry_msgs::Pose state;
    state.position.x = vehicle_state.x;
    state.position.y = vehicle_state.y;
    state.orientation.z = vehicle_state.z;
    state.orientation.y = vehicle_state.w;
    return state;
}

void BehaviourPlanning::run()
{
    using namespace std::chrono_literals;
    while (running_) {
        auto vehicle_state = this->context<RobotStateMachine>().getVehicleState();
        auto state = convertVehicleState(vehicle_state);
        if(distance(goal_, state) < 0.2) {
            post_event(GoalReached());
        }  
        std::this_thread::sleep_for(100ms);
    }
}

void BehaviourPlanning::stop()
{
     running_ = false;
}

BehaviourPlanning::~BehaviourPlanning() {
    stop();
    if (planning_thread_.joinable()) {
        planning_thread_.join();
    }
}