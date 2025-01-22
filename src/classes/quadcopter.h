#ifndef QUAD_NODE_H
#define QUAD_NODE_H

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "ros2_tools/msg/lidar_pose.hpp"
#include <std_msgs/msg/string.hpp>

class quadcopter;
class target;

class quadcopter : public rclcpp::Node {
public:
    float x, y, z, yaw; // 全局位置

    quadcopter();
    void quadcopter_init();

private:
    std::shared_ptr<std::thread> spin_thread;
    
    std::shared_ptr<rclcpp::Rate> rate;
    rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr lidar_sub;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pos_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_pub;
    
    std::shared_ptr<ros_tools::msg::LidarPose> lidar_pos;
    std::shared_ptr<geometry_msgs::msg::Twist> target_pos;

    friend class target;

    void lidar_pose_cb(const ros2_tools::msg::LidarPose::SharedPtr msg);
};
#endif
