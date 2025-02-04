#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <math.h>
#include "quadcopter.h"
#include "target.hpp"

#define pi 3.14

class quadcopter;
class target;

class flight_controller : public rclcpp::Node {
public:
    flight_controller(std::shared_ptr<quadcopter> quad_node);
    void fly_to_target(std::shared_ptr<quadcopter> quad_node, target* target); // target定点移动

private:
    target* current_target;
    std::shared_ptr<rclcpp::Rate> rate;
    std::shared_ptr<quadcopter> quad_node;

    // 自身位置检查，distance为误差默认0.1
    bool pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, 
                                    target* target, 
                                    double distance = 0.1);
    // 重载严格检查，多维误差
    bool pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, 
                                    target* target, 
                                    double distance_x, 
                                    double distance_y, 
                                    double distance_z);
};
#endif
