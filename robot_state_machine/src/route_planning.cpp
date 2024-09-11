#include "../include/robot_state_machine/running.h"
#include "../include/robot_state_machine/move.h"
#include "../include/robot_state_machine/error.h"

RoutePlanningState::RoutePlanningState(my_context ctx) : my_base(ctx) {
    this->context<RobotStateMachine>().publishState("ROUTE PLANNING");
    //update the route vector in the running context
    post_event(RoutePlanned());   
}

boost::statechart::result RoutePlanningState::react(const RoutePlanned& route_planned) {
    return transit<Move>(); 
}

boost::statechart::result RoutePlanningState::react(const ErrorDetected &error)
{
    return transit<Error>();
}