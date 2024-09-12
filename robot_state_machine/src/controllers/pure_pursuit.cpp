#include "../../include/robot_state_machine/controllers/pure_pursuit.h"

#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_datatypes.h>

PurePursuit::PurePursuit(ros::NodeHandle &nh) 
                        : rate(20), look_ahead(6.0), wheelbase(1.75), goal(0) {
    ackermann_pub = nh.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 1);
    ackermann_msg.steering_angle_velocity = 0.0;
    ackermann_msg.acceleration = 0.0;
    ackermann_msg.jerk = 0.0;
    ackermann_msg.speed = 0.0;
    ackermann_msg.steering_angle = 0.0;
}

double PurePursuit::dist(const std::pair<double, double> &p1, const std::pair<double, double> &p2) {
    return std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
}

double PurePursuit::findAngle(const tf::Vector3 &v1, const tf::Vector3 &v2) {
    double cosang = v1.dot(v2);
    double sinang = v1.cross(v2).length();
    return std::atan2(sinang, cosang);
}

void PurePursuit::start(const std::vector<double>& path_points_x, const std::vector<double>& path_points_y, const std::vector<double>& path_points_yaw) {
    path_points_x_ = path_points_x;
    path_points_y_ = path_points_y;
    path_points_yaw_= path_points_yaw;
    dist_arr.resize(path_points_x.size());
}

void PurePursuit::update(const VehicleState& vehicle_state) {
        tf::Quaternion q(0.0, 0.0, vehicle_state.z, vehicle_state.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        for (size_t i = 0; i < path_points_x_.size(); ++i) {
            dist_arr[i] = dist({path_points_x_[i], path_points_y_[i]}, {vehicle_state.x, vehicle_state.y});
        }

        std::vector<size_t> goal_indices;
        for (size_t i = 0; i < dist_arr.size(); ++i) {
            if (dist_arr[i] < look_ahead + 0.3 && dist_arr[i] > look_ahead - 0.3) {
                goal_indices.push_back(i);
            }
        }

        for (size_t idx : goal_indices) {
            tf::Vector3 v1(path_points_x_[idx] - vehicle_state.x, path_points_y_[idx] - vehicle_state.y, 0);
            tf::Vector3 v2(std::cos(yaw), std::sin(yaw), 0);
            double angle = findAngle(v1, v2);
            if (std::abs(angle) < M_PI / 2) {
                goal = idx;
                break;
            }
        }

        double L = dist_arr[goal];

        // Transforming the goal point into the vehicle coordinate frame
        double gvcx = path_points_x_[goal] - vehicle_state.x;
        double gvcy = path_points_y_[goal] - vehicle_state.y;
        double goal_x_veh_coord = gvcx * std::cos(yaw) + gvcy * std::sin(yaw);
        double goal_y_veh_coord = gvcy * std::cos(yaw) - gvcx * std::sin(yaw);

        double alpha = path_points_yaw_[goal] - yaw;
        double k = 0.285;
        double angle_i = std::atan((2 * k * wheelbase * std::sin(alpha)) / L);
        double angle = angle_i * 2;
        angle = std::max(-0.61, std::min(angle, 0.61));

        double ct_error = std::sin(alpha) * L;
        ROS_INFO_STREAM("Crosstrack Error: " << ct_error);

        ackermann_msg.speed = 2.8;
        ackermann_msg.steering_angle = angle;
        ackermann_pub.publish(ackermann_msg);
}

