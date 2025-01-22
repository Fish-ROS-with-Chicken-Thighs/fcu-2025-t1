#include<quadcopter.h>
#include<target.h>

#define pi 3.14

// 构造
target::target(float x, float y, float z, float yaw, quadcopter* quad_node) : Node("simple_target_node"), reached(false), quad_node(quad_node), rate(std::make_shared<rclcpp::Rate>(20)) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->yaw = yaw;
}

// 非阻塞式移动
void target::drive_to_target(){
    do {quad_node->target_pos->linear.x = this->x;
        quad_node->target_pos->linear.y = this->y;
        quad_node->target_pos->angular.z = this->yaw;
        quad_node->target_pos->linear.z = 1; // 标志位
    
        quad_node->pos_pub->publish(*quad_node->target_pos);
        //TODO：正在移动到点，可以添加突发事件
        rate->sleep();
    } while (!pos_check(quad_node->lidar_pos));
    quad_node->pos_pub->publish(*quad_node->target_pos);
}

// 自身位置检查，distance为误差
bool target::pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, double distance) {
    return reached || (reached = std::sqrt(std::pow(lidar_pos->x - x, 2) +
                                            std::pow(lidar_pos->y - y, 2) + 
                                            std::pow(lidar_pos->z - z, 2)) < distance &&
                                std::abs(lidar_pose_data->yaw - yaw) < 0.1);
}

//重载严格检查，多误差
bool target::pos_check(std::shared_ptr<ros2_tools::msg::LidarPose> lidar_pos, double distance_x, double distance_y, double distance_z) {
    return reached || (reached = std::abs(lidar_pos->x - x) < distance_x &&
                                std::abs(lidar_pos->y - y) < distance_y &&
                                std::abs(lidar_pos->z - z) < distance_z && 
                                std::abs(lidar_pos->yaw - yaw) < 0.1);
}
