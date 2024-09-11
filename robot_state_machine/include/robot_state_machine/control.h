// #ifndef CONTROL_STATE_H
// #define CONTROL_STATE_H

// #include "move.h"

// #include <boost/statechart/state.hpp>


// struct Control: boost::statechart::state<Control, Move::orthogonal< 1 > > {
//     Control(my_context ctx);
//     ~Control(){}
//     typedef boost::mpl::list<
//         boost::statechart::custom_reaction<ErrorDetected>
//     > reactions;
//     boost::statechart::result react(const ErrorDetected& error);
// };

// #endif