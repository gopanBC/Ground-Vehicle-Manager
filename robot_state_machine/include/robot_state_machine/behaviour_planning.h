// #ifndef BEHAVIOUR_PLANNING_STATE_H
// #define BEHAVIOUR_PLANNING_STATE_H

// #include "move.h"

// #include <boost/statechart/state.hpp>


// struct BehaviourPlanning: boost::statechart::state<BehaviourPlanning, Move::orthogonal< 0 > > {
//     BehaviourPlanning(my_context ctx);
//     ~BehaviourPlanning(){}
//     typedef boost::mpl::list<
//         boost::statechart::custom_reaction<ErrorDetected>
//     > reactions;
//     boost::statechart::result react(const GoalReached& goal_rea);
//     boost::statechart::result react(const ErrorDetected& error);
// };

// #endif