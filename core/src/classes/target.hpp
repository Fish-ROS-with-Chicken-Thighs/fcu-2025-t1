#ifndef TARGET_HPP
#define TARGET_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

// 目标点类
class target {
public:
    geometry_msgs::msg::PoseStamped pose;
    bool reached;

    target(float x, float y, float z, float yaw) : reached(false) {
        pose.header.stamp = rclcpp::Clock().now(); // 时间戳
        pose.header.frame_id = "map"; // map代表全局坐标系

        // 设定位置
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        // 转换yaw->四元数
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
    }

    // 允许 target 直接转换为 PoseStamped
    operator geometry_msgs::msg::PoseStamped&() {
        return pose;
    }

    // 允许 target 直接转换为 const PoseStamped
    operator const geometry_msgs::msg::PoseStamped&() const {
        return pose;
    }

    float get_x() const { return pose.pose.position.x; }
    float get_y() const { return pose.pose.position.y; }
    float get_z() const { return pose.pose.position.z; }
    float get_yaw() const { return std::atan2(2.0 * (pose.pose.orientation.z * pose.pose.orientation.w), 1.0 - 2.0 * (pose.pose.orientation.z * pose.pose.orientation.z)); }
};
#endif
