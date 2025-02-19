#ifndef TARGET_HPP
#define TARGET_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

// 目标点类
class target {
public:
    geometry_msgs::msg::PoseStamped pose_stamped;
    bool reached;

    target() : reached(false) {}
    
    target(float x, float y, float z, float yaw) : reached(false) {
        pose_stamped.header.stamp = rclcpp::Clock().now(); // 时间戳
        pose_stamped.header.frame_id = "map"; // map代表全局坐标系
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = z;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = std::sin(yaw / 2.0);
        pose_stamped.pose.orientation.w = std::cos(yaw / 2.0);
    }

    float get_x() const { return pose_stamped.pose.position.x; }
    float get_y() const { return pose_stamped.pose.position.y; }
    float get_z() const { return pose_stamped.pose.position.z; }
    float get_yaw() const { return std::atan2(2.0 * (pose_stamped.pose.orientation.z * pose_stamped.pose.orientation.w), 
                                        1.0 - 2.0 * (pose_stamped.pose.orientation.z * pose_stamped.pose.orientation.z)); }

    // 允许 target 直接转换为 PoseStamped
    operator geometry_msgs::msg::PoseStamped&() { return pose_stamped; }
    operator const geometry_msgs::msg::PoseStamped&() const { return pose_stamped; }
};
#endif
