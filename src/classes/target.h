#ifndef TARGETS_NODE_H
#define TARGETS_NODE_H

#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "ros2_tools/msg/lidar_pose.hpp"

#define pi 3.14

class quadcopter;

//--------------------------------------------------------------------------------------------------
// 目标点类
class target : public rclcpp::Node {
public:
    float x, y, z, yaw;
    bool reached;

    target(quadcopter* quad_node);
    target(float x, float y, float z, float yaw, quadcopter* quad_node);

    void drive_to_target();
    bool pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, double distance = 0.1);
    bool pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, double distance_x, double distance_y, double distance_z);

private:
    quadcopter* quad_node;
    std::shared_ptr<rclcpp::Rate> rate;
};
