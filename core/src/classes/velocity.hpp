#ifndef VELOCITY_HPP
#define VELOCITY_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>

// 速度类
class velocity {
public:
    geometry_msgs::msg::TwistStamped twist_stamped;

    velocity(float v_x, float v_y, float v_z, float v_roll=0, float v_pitch=0, float v_yaw=0) {
        twist_stamped.twist.linear.x = v_x;
        twist_stamped.twist.linear.y = v_y;
        twist_stamped.twist.linear.z = v_z;
        twist_stamped.twist.angular.x = v_roll;
        twist_stamped.twist.angular.y = v_pitch;
        twist_stamped.twist.angular.z = v_yaw;
    }

    float get_vx() const { return twist_stamped.twist.linear.x; }
    float get_vy() const { return twist_stamped.twist.linear.y; }
    float get_vz() const { return twist_stamped.twist.linear.z; }
    float get_vroll() const { return twist_stamped.twist.angular.x; }
    float get_vpitch() const { return twist_stamped.twist.angular.y; }
    float get_vyaw() const { return twist_stamped.twist.angular.z; }

    // 允许 velocity 直接转换为 TwistStamped
    operator geometry_msgs::msg::TwistStamped&() { return twist_stamped; }
    operator const geometry_msgs::msg::TwistStamped&() const { return twist_stamped; }
};
#endif
