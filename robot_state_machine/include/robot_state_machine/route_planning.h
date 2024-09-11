// #ifndef ROUTE_PLANNING_STATE_H
// #define ROUTE_PLANNING_STATE_H

// #include "robot_state_machine.h"
// #include "running.h"

// #include <boost/statechart/state.hpp>


// // RoutePlanningState definition
// struct RoutePlanningState : public boost::statechart::state<RoutePlanningState, Running> {
//     RoutePlanningState(my_context ctx);
//     typedef boost::mpl::list<
//         boost::statechart::custom_reaction<ErrorDetected>,
//         boost::statechart::custom_reaction<RoutePlanned>
//     > reactions;
//     boost::statechart::result react(const RoutePlanned& error);
//     boost::statechart::result react(const ErrorDetected& error);

// };

// #endif // ROUTE_PLANNING_STATE_H
