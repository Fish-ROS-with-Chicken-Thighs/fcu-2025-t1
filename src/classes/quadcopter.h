#ifndef QUAD_NODE_H
#define QUAD_NODE_H

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <std_msgs/msg/string.hpp>

#include <mavros_msgs/srv/command_bool.h>
#include <mavros_msgs/srv/command_long.h>
#include <mavros_msgs/srv/set_mode.h>
#include <mavros_msgs/msg/state.h>

#include "ros2_tools/msg/lidar_pose.hpp"
#include "flight_controller.h"

class quadcopter : public rclcpp::Node {
public:
    float x, y, z, yaw; // 全局位置
    float vx, vy, vz; // 速度
    float ax, ay, az; // 加速度
    float roll, pitch; // 姿态角
    float dr, dp; // 角速度

    mavros_msgs::msg::State current_state;

    quadcopter();
    void quad_init(); // 初始化quad节点控制流程（可能进行树结构改良）
    void start_spin_thread(); // 注册shutdown回调（全局）并创建spin线程
    void perform_pre_flight_checks(); // 起飞前检查
    void main_loop(); // 主循环

private:
    std::shared_ptr<std::thread> spin_thread;
    
    std::shared_ptr<rclcpp::Rate> rate;
    rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr lidar_sub;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pos_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_pub;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    
    std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> target_pos;

    friend class target;

    void lidar_pose_cb(const ros2_tools::msg::LidarPose::SharedPtr msg); // 雷达数据回调

protected:
    std::shared_ptr<flight_controller> flight_controller_;
};
#endif
