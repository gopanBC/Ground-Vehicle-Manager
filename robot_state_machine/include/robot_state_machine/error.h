#ifndef ERROR_STATE_H
#define ERROR_STATE_H

#include "robot_state_machine.h"

#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

// Error state definition
struct Error : public boost::statechart::state<Error, RobotStateMachine> {
    Error(my_context ctx);
    typedef boost::mpl::list<
        //boost::statechart::transition<ErrorCleared, Idle>
        boost::statechart::custom_reaction<ErrorCleared>> reactions;
    boost::statechart::result react(const ErrorCleared& err_cleared);
    ~Error();
};

#endif // ERROR_STATE_H
