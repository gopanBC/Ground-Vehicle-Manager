#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "robot_state_machine.h"

#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>


//struct Error;

// Idle state definition
struct Idle : public boost::statechart::state<Idle, RobotStateMachine> {
    Idle(my_context ctx);
    typedef boost::mpl::list<
        //boost::statechart::transition<StartRunning, Running>
        //boost::statechart::transition<ErrorDetected, Error>
        boost::statechart::custom_reaction<StartRunning>, boost::statechart::custom_reaction<ErrorDetected>,
        boost::statechart::custom_reaction<Routes>
    > reactions;
    boost::statechart::result react(const StartRunning& running);
    boost::statechart::result react(const ErrorDetected& error);
    boost::statechart::result react(const Routes& routes);
    ~Idle();
};

#endif // IDLE_STATE_H
