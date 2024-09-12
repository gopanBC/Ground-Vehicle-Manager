#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "controller_interface.h"

#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <tf/tf.h>

class PurePursuit : public ControllerInterface {
public:
    PurePursuit(ros::NodeHandle &nh);
    virtual ~PurePursuit() = default;

    void start(const std::vector<double>& path_points_x, const std::vector<double>& path_points_y, const std::vector<double>& path_points_yaw) override;
    void update(const VehicleState& vehicle_state) override;

private:

    double dist(const std::pair<double, double> &p1, const std::pair<double, double> &p2);
    double findAngle(const tf::Vector3 &v1, const tf::Vector3 &v2);

    ros::Publisher ackermann_pub;
    ackermann_msgs::AckermannDrive ackermann_msg;

    ros::Rate rate;
    double look_ahead;
    double wheelbase;
    size_t goal;

    std::vector<double> path_points_x_;
    std::vector<double> path_points_y_;
    std::vector<double> path_points_yaw_;

    std::vector<double> dist_arr;
};

#endif // PURE_PURSUIT_H
