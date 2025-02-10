#include "flight_controller.h"

flight_controller::flight_controller(std::shared_ptr<quadcopter> quad_node) : Node("flight_controller_node"), quad_node(quad_node), rate(std::make_shared<rclcpp::Rate>(20)) {}

// target定点移动
void flight_controller::fly_to_target(target* target) {
    do {
        target->pose_stamped.header.stamp = rclcpp::Clock().now();
        quad_node->pos_pub->publish(target->pose_stamped);
        rate->sleep();
    } while (!pos_check(target));
}

// 自身位置检查，distance为误差默认0.1
bool flight_controller::pos_check(target* target, float distance) {
    return target->reached || (target->reached = std::sqrt(std::pow(quad_node->lidar_pos->x - target->get_x(), 2) +
                                                std::pow(quad_node->lidar_pos->y - target->get_y(), 2) + 
                                                std::pow(quad_node->lidar_pos->z - target->get_z(), 2)) < distance &&
                                                std::abs(quad_node->lidar_pos->yaw - target->get_yaw()) < 0.1);}

// 严格检查，多维误差
bool flight_controller::pos_check(target* target, float distance_x, float distance_y, float distance_z) {
    return target->reached || (target->reached = std::abs(quad_node->lidar_pos->x - target->get_x()) < distance_x &&
                                                std::abs(quad_node->lidar_pos->y - target->get_y()) < distance_y &&
                                                std::abs(quad_node->lidar_pos->z - target->get_z()) < distance_z && 
                                                std::abs(quad_node->lidar_pos->yaw - target->get_yaw()) < 0.1);}

// velocity速度飞行，单次发布
void flight_controller::fly_by_velocity(velocity* velocity) {
    velocity->twist_stamped.header.stamp = rclcpp::Clock().now();
    quad_node->vel_pub->publish(velocity->twist_stamped);
    rate->sleep();
}

// 路径航点飞行，已兼容target版本
void flight_controller::fly_by_path(path* path) {
    target waypoint;
    if (path->get_next_waypoint(waypoint)) {
        fly_to_target(&waypoint);
    } else {
        RCLCPP_INFO(this->get_logger(), "航点已全部执行完毕");
    }
}
