#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class LidarToPx4Bridge : public rclcpp::Node {
public:
  LidarToPx4Bridge() : Node("lidar_to_px4_bridge") {
    // 订阅 Odometry 消息
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 10, std::bind(&LidarToPx4Bridge::odomCallback, this, std::placeholders::_1));

    // 发布给 PX4 飞控
    position_target_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 将 Odometry 消息转换为 PositionTarget 消息
    auto position_target = mavros_msgs::msg::PositionTarget();

    // 设置时间戳
    position_target.header.stamp = this->now();
    position_target.header.frame_id = "map";

    // 位置
    position_target.position.x = msg->pose.pose.position.x;
    position_target.position.y = msg->pose.pose.position.y;
    position_target.position.z = msg->pose.pose.position.z;

    // 速度
    position_target.velocity.x = msg->twist.twist.linear.x;
    position_target.velocity.y = msg->twist.twist.linear.y;
    position_target.velocity.z = msg->twist.twist.linear.z;

    // 加速度（可选）
    position_target.acceleration_or_force.x = 0.0;
    position_target.acceleration_or_force.y = 0.0;
    position_target.acceleration_or_force.z = 0.0;

    // 角度（可选）
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    position_target.yaw = yaw;

    // 设置坐标系和类型掩码
    position_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    position_target.type_mask = 0;  // 使用所有字段
    position_target_pub->publish(position_target);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_target_pub;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarToPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}