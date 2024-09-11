#ifndef RUNNING_STATE_H
#define RUNNING_STATE_H

#include "robot_state_machine.h"

#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

struct RoutePlanningState;

// Running state definition
struct Running : public boost::statechart::state<Running, RobotStateMachine, RoutePlanningState> {
    Running(my_context ctx);
    typedef boost::mpl::list<
        //boost::statechart::transition<ErrorDetected, Error>
        boost::statechart::custom_reaction<ErrorDetected>> reactions;
    boost::statechart::result react(const ErrorDetected& error);
    ~Running(){}
};

// RoutePlanningState definition
struct RoutePlanningState : public boost::statechart::state<RoutePlanningState, Running> {
    RoutePlanningState(my_context ctx);
    typedef boost::mpl::list<
        boost::statechart::custom_reaction<ErrorDetected>,
        boost::statechart::custom_reaction<RoutePlanned>
    > reactions;
    boost::statechart::result react(const RoutePlanned& error);
    boost::statechart::result react(const ErrorDetected& error);
};

#endif // RUNNING_STATE_H
