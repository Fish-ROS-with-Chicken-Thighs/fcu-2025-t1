#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "quadcopter.h"
#include "target.hpp"
#include "velocity.hpp"
#include "path.hpp"

#define pi 3.14

class quadcopter;
class target;
class velocity;
class path;

class flight_controller : public rclcpp::Node {
public:
    flight_controller(std::shared_ptr<quadcopter> quad_node);
    void fly_to_target(target* target); // target定点移动
    void fly_by_velocity(velocity* velocity); // 速度发布飞行（单次）
    void fly_by_path(); // 路径航点飞行

private:
    std::shared_ptr<quadcopter> quad_node;
    std::shared_ptr<rclcpp::Rate> rate;

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
