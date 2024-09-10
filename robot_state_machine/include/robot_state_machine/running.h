#ifndef RUNNING_STATE_H
#define RUNNING_STATE_H

#include "robot_state_machine.h"

#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

// Running state definition
struct Running : public boost::statechart::state<Running, RobotStateMachine> {
    Running(my_context ctx);
    typedef boost::mpl::list<
        //boost::statechart::transition<ErrorDetected, Error>
        boost::statechart::custom_reaction<ErrorDetected>> reactions;
     boost::statechart::result react(const ErrorDetected& error);
    ~Running(){}
};

#endif // RUNNING_STATE_H
