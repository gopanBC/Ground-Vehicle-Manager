#include "../include/robot_state_machine/robot_state_machine.h"

RobotStateMachine::RobotStateMachine() {
    ros::NodeHandle nh;
    state_publisher = nh.advertise<std_msgs::String>("robot_state", 1, true);
    state_subscriber = nh.subscribe("/gazebo/model_states", 1, &RobotStateMachine::vehicleStateCB, this);
}

void RobotStateMachine::publishState(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    state_publisher.publish(msg);
}

void RobotStateMachine::saveRoutes(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

void RobotStateMachine::getRoutes(std::vector<double> &x, std::vector<double> &y, std::vector<double> &yaw)
{
    x = x_;
    y = y_;
    yaw = yaw_;
}

void RobotStateMachine::setVehicleState(const VehicleState& new_state) {
    // VehicleState state;
    // state.x = new_pose.position.x;
    // state.y = new_pose.position.y;
    // state.z = new_pose.orientation.z;
    // state.w = new_pose.orientation.w;
    atomic_state.store(new_state, std::memory_order_relaxed);
}

void RobotStateMachine::vehicleStateCB(const gazebo_msgs::ModelStates::ConstPtr& state_info)
{
    const auto& vehicle_state_msg = state_info->pose[7];

    // copying ros msg data to the trivially copyable state structure.
    VehicleState vehicle_state;
    vehicle_state.x = vehicle_state_msg.position.x;
    vehicle_state.y = vehicle_state_msg.position.y;
    vehicle_state.z = vehicle_state_msg.orientation.z;
    vehicle_state.w = vehicle_state_msg.orientation.w;
    setVehicleState(vehicle_state);
}

VehicleState RobotStateMachine::getVehicleState()
{
    return atomic_state.load(std::memory_order_relaxed);
}

void RobotStateMachine::setNewGoal(const geometry_msgs::Pose& new_goal) {
    goal_ = new_goal;
}

geometry_msgs::Pose& RobotStateMachine::getGoal() {
    return goal_;
}