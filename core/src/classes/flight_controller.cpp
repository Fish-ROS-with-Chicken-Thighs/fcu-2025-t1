#include "flight_controller.h"

flight_controller::flight_controller(std::shared_ptr<quadcopter> quad_node) : Node("flight_controller_node"), quad_node(quad_node), rate(std::make_shared<rclcpp::Rate>(20)) {}

// target定点移动
void flight_controller::fly_to_target(target* target) {
    do {
        quad_node->pos_pub->publish(target->pose);
        rate->sleep();
    } while (!pos_check(quad_node->lidar_pos, target));
}

using namespace ros2_tools::msg;
// 自身位置检查，distance为误差默认0.1
bool flight_controller::pos_check(std::shared_ptr<LidarPose> lidar_pos, target* target, float distance) {
    return target->reached || (target->reached = std::sqrt(std::pow(lidar_pos->x - target->get_x(), 2) +
                                            std::pow(lidar_pos->y - target->get_y(), 2) + 
                                            std::pow(lidar_pos->z - target->get_z(), 2)) < distance &&
                                            std::abs(lidar_pos->yaw - target->get_yaw()) < 0.1);}

// 严格检查，多维误差
bool flight_controller::pos_check(std::shared_ptr<LidarPose> lidar_pos, target* target, float distance_x, float distance_y, float distance_z) {
    return target->reached || (target->reached = std::abs(lidar_pos->x - target->get_x()) < distance_x &&
                                                std::abs(lidar_pos->y - target->get_y()) < distance_y &&
                                                std::abs(lidar_pos->z - target->get_z()) < distance_z && 
                                                std::abs(lidar_pos->yaw - target->get_yaw()) < 0.1);}

// 速度发布飞行（单次）,旧版本
void flight_controller::fly_by_velocity(velocity* velocity) {
    quad_node->move_vel->twist.linear.x = velocity->vx;
    quad_node->move_vel->twist.linear.y = velocity->vy;
    quad_node->move_vel->twist.linear.z = velocity->vz;
    quad_node->move_vel->twist.angular.x = velocity->dy;
    quad_node->move_vel->twist.angular.y = velocity->dp;
    quad_node->move_vel->twist.angular.z = velocity->dr;

    quad_node->vel_pub->publish(*quad_node->move_vel);
}

// 路径航点飞行，未兼容target版本
void flight_controller::fly_by_path(path* path) {
    geometry_msgs::msg::PoseStamped waypoint;
    if (path->get_next_waypoint(waypoint)) {
        quad_node->pos_pub->publish(waypoint);
        RCLCPP_INFO(this->get_logger(), "发送航点: x=%.2f, y=%.2f, z=%.2f",
                    waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
    } else {
        RCLCPP_INFO(this->get_logger(), "航点已全部执行完毕");
    }
}
