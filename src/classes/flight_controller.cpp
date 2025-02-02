#include "flight_controller.h"

flight_controller::flight_controller(std::weak_ptr<quadcopter> quad_node) : Node("flight_controller_node"), quad_node(quad_node), rate(std::make_shared<rclcpp::Rate>(20)) {}

// target定点移动
void flight_controller::fly_to_target(std::weak_ptr<quadcopter> quad_node, target* target) {
    if (auto quad_ptr = quad_node.lock()) {
        quad_ptr->target_pos->pose.position.x = target->x;
        quad_ptr->target_pos->pose.position.y = target->y;
        quad_ptr->target_pos->pose.position.z = target->z;
        quad_ptr->target_pos->pose.orientation.x = 0.0;
        quad_ptr->target_pos->pose.orientation.y = 0.0;
        quad_ptr->target_pos->pose.orientation.z = sin(target->yaw / 2);
        quad_ptr->target_pos->pose.orientation.w = cos(target->yaw / 2);

        do {
            quad_ptr->pos_pub->publish(*quad_ptr->target_pos);
            rate->sleep();
        } while (!pos_check(quad_ptr->lidar_pos, target));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to lock quadcopter object.");
    }
}

// 自身位置检查，distance为误差默认0.1
bool flight_controller::pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, target* target, double distance) {
    return target->reached || (target->reached = std::sqrt(std::pow(lidar_pos->x - target->x, 2) +
                                            std::pow(lidar_pos->y - target->y, 2) + 
                                            std::pow(lidar_pos->z - target->z, 2)) < distance &&
                                            std::abs(lidar_pos->yaw - target->yaw) < 0.1);
}

// 重载严格检查，多维误差
bool flight_controller::pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, target* target, double distance_x, double distance_y, double distance_z) {
    return target->reached || (target->reached = std::abs(lidar_pos->x - target->x) < distance_x &&
                                                std::abs(lidar_pos->y - target->y) < distance_y &&
                                                std::abs(lidar_pos->z - target->z) < distance_z && 
                                                std::abs(lidar_pos->yaw - target->yaw) < 0.1);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto flight_controller_node = std::make_shared<flight_controller>(quad_node);
    rclcpp::spin(flight_controller_node);
    rclcpp::shutdown();
    return 0;
}