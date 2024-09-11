#include "../include/robot_state_machine/move.h"

Move::Move(my_context ctx) : my_base(ctx) {
    this->context<RobotStateMachine>().publishState("MOVE");
}

std::vector<geometry_msgs::Pose>& Move::getBehaviourPlan()
{
    return behaviour_plan_;
}

std::mutex& Move::getBehaviourPlanMutex()
{
    return behaviour_plan_mutex_;
}
