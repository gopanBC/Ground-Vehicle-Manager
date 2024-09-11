#ifndef MOVE_STATE_H
#define MOVE_STATE_H

#include "running.h"

#include <boost/statechart/state.hpp>

#include <vector>

#include <geometry_msgs/Pose.h>

struct BehaviourPlanning;
struct Control;

struct Move: boost::statechart::state< Move, Running, boost::mpl::list< BehaviourPlanning, Control > > {

private:
    std::vector<geometry_msgs::Pose> behaviour_plan_;
    std::mutex behaviour_plan_mutex_;

public:
    std::vector<geometry_msgs::Pose>& getBehaviourPlan();
    std::mutex& getBehaviourPlanMutex();
    Move(my_context ctx);
};

struct BehaviourPlanning: boost::statechart::state<BehaviourPlanning, Move::orthogonal< 0 > > {
    BehaviourPlanning(my_context ctx);
    ~BehaviourPlanning();
    typedef boost::mpl::list<
        boost::statechart::custom_reaction<ErrorDetected>,
        boost::statechart::custom_reaction<GoalReached>,
        boost::statechart::custom_reaction<Stop>
    > reactions;
    boost::statechart::result react(const GoalReached& goal_reached);
    boost::statechart::result react(const ErrorDetected& error);
    boost::statechart::result react(const Stop& stop);

private:
    std::thread planning_thread_;
    std::atomic<bool> running_;
    std::vector<geometry_msgs::Pose> behaviour_plan_; // The shared behavior plan
    std::mutex behaviour_plan_mutex_; // Mutex for thread-safe access

    void run();
    void stop();

};

struct Control: boost::statechart::state<Control, Move::orthogonal< 1 > > {
    Control(my_context ctx);
    ~Control();
    typedef boost::mpl::list<
        boost::statechart::custom_reaction<ErrorDetected>,
        boost::statechart::custom_reaction<Stop>
    > reactions;
    boost::statechart::result react(const ErrorDetected& error);
    boost::statechart::result react(const Stop& stop);

private:
    std::thread control_thread_;
    std::atomic<bool> running_;
    std::vector<geometry_msgs::Pose> behaviour_plan_; // The shared behavior plan
    std::mutex behaviour_plan_mutex_; // Mutex for thread-safe access

    void run();
    void stop();
};

#endif