#include<quadcopter.h>

quadcopter::quadcopter() : Node("quad_node") {
    rate = std::make_shared<rclcpp::Rate>(20.0);
    lidar_sub = this->create_subscription<ros2_tools::msg::LidarPose>("lidar_data", 10, std::bind(&quadcopter::lidar_pose_cb, this, std::placeholders::_1));
    
    pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("position", 10);

    arming_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    command_client = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
    set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    lidar_pos = std::make_shared<ros2_tools::msg::LidarPose>();
    target_pos = std::make_shared<geometry_msgs::msg::PoseStamped>();
    RCLCPP_INFO(this->get_logger(), "quadcopter init");

    flight_ctrl = std::make_shared<flight_controller>(std::static_pointer_cast<quadcopter>(shared_from_this()));
    RCLCPP_INFO(this->get_logger(), "flight_ctrl init");
}

// 初始化quad节点控制流程（可能进行树结构改良）
void quadcopter::quad_init() {
    start_spin_thread();
    pre_flight_checks_loop();
    main_loop();
}

// 注册shutdown回调（全局），并创建spin线程处理回调
void quadcopter::start_spin_thread() {
    rclcpp::on_shutdown([this]() {
        if (spin_thread && spin_thread->joinable()) {
            spin_thread->join();
        }
    });

    spin_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
}

// 起飞前检查
void quadcopter::pre_flight_checks_loop() {
    mavros_msgs::srv::CommandBool::Request arm_cmd;
    arm_cmd.value = true;
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    *arm_request = arm_cmd;

    mavros_msgs::srv::SetMode::Request offb_set_mode;
    offb_set_mode.custom_mode = "OFFBOARD";
    auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    *mode_request = offb_set_mode;

    rclcpp::Time last_request = this->now();
    while (rclcpp::ok()) {
        // 定时检查是否解锁
        if (!current_state.armed && (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
            if (arming_client->async_send_request(arm_request).valid()) {
                RCLCPP_INFO(this->get_logger(), "arming...");
            }
            last_request = this->now();
        } 
        // 定时尝试OFFBOARD
        else if (current_state.mode != "OFFBOARD" && (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
            if (set_mode_client->async_send_request(mode_request).valid()) {
                RCLCPP_INFO(this->get_logger(), "armed and OFFBOARDING...");
                target simp(0, 0, 0.5, 0);
                flight_ctrl->fly_to_target(std::static_pointer_cast<quadcopter>(shared_from_this()), &simp);
            }
            last_request = this->now();
        }
        // 如果已解锁，并offbard，结束循环
        if (current_state.armed && current_state.mode == "OFFBOARD") {
            RCLCPP_INFO(this->get_logger(), "armed and OFFBOARD success!");
            break;
        }
        rate->sleep();
    }
}

// 主循环
void quadcopter::main_loop() {
    while (rclcpp::ok()) {
        //int mode = 0;
        rate->sleep();
    }
}

// 雷达数据回调
void quadcopter::lidar_pose_cb(const ros2_tools::msg::LidarPose::SharedPtr msg) {
    lidar_pos = msg;
    x = msg->x;
    y = msg->y;
    z = msg->z;
    yaw = msg->yaw;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto quad_node = std::make_shared<quadcopter>();
    quad_node->quad_init();
    rclcpp::shutdown();
    return 0;
}