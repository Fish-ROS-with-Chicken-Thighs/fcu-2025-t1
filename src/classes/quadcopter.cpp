#include<quadcopter.h>
#include<target.h>

class target;

quadcopter::quadcopter() : Node("quad_node") {
    rate = std::make_shared<rclcpp::Rate>(20.0);
    lidar_sub = this->create_subscription<ros2_tools::msg::LidarPose>("lidar_data", 10, std::bind(&car::lidar_pose_cb, this, std::placeholders::_1));
    
    pos_pub = this->create_publisher<geometry_msgs::msg::Twist>("position", 10);
    vel_pub = this->create_publisher<geometry_msgs::msg::Vector3>("vel", 10);

    lidar_pos = std::make_shared<ros2_tools::msg::LidarPose>();
    target_pos = std::make_shared<geometry_msgs::msg::>();

    RCLCPP_INFO(this->get_logger(), "quadcopter init");
}

// 初始化quad节点控制流程
void quadcopter::quad_init() {
    // 创建spin线程
    spin_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
    
    // 起飞前检查

    // 主循环
    while(rclcpp::ok()) {
        rate->sleep();
    }
}

// 激光雷达数据回调
void quadcopter::lidar_pose_cb(const ros2_tools::msg::LidarPose::SharedPtr msg) {
    lidar_pos = msg;
    x = msg->x;
    y = msg->y;
    z = msg->z;
    yaw = msg->yaw;
}