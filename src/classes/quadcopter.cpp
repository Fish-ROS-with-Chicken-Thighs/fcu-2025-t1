#include<quadcopter.h>

quadcopter::quadcopter() : Node("quad_node") {
    rate = std::make_shared<rclcpp::Rate>(20.0);
    lidar_sub = this->create_subscription<ros2_tools::msg::LidarPose>("lidar_data", 10, std::bind(&car::lidar_pose_cb, this, std::placeholders::_1));
    
    pos_pub = this->create_publisher<geometry_msgs::msg::Twist>("position", 10);
    vel_pub = this->create_publisher<geometry_msgs::msg::Vector3>("vel", 10);

    arming_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    command_client = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");
    set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    lidar_pos = std::make_shared<ros2_tools::msg::LidarPose>();
    target_pos = std::make_shared<geometry_msgs::msg::PoseStamped>();

    RCLCPP_INFO(this->get_logger(), "quadcopter init");
}

// 初始化quad节点控制流程（可能进行树结构改良）
void quadcopter::quad_init() {
    start_spin_thread();
    pre_flight_checks_loop();
    main_loop();
}

// 注册shutdown回调（全局）并创建spin线程
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
void quadcopter::perform_pre_flight_checks() {
    mavros_msgs::srv::SetMode::Request offb_set_mode;
    offb_set_mode.custom_mode = "OFFBOARD";

    mavros_msgs::srv::CommandBool::Request arm_cmd;
    arm_cmd.value = true;
    
    rclcpp::Time last_request = this->now();
    while (rclcpp::ok()) {
        if (!current_state.armed && (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
            if (arming_client->call(arm_cmd) && arm_cmd.response.success) {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed");
            }
            last_request = this->now();
        } else if (current_state.mode != "OFFBOARD" && (this->now() - last_request > rclcpp::Duration::from_seconds(1.0))) {
            if (set_mode_client->call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Offboard enabled");
                target simp(0, 0, 0.5, 0, this);
                simp.drive_to_target();
            }
            last_request = this->now();
        }
        if (current_state.armed && current_state.mode == "OFFBOARD") {
            break;
        }
        rate->sleep();
    }
}

// 主循环
void quadcopter::main_loop() {
    while (rclcpp::ok()) {
        int mode = 0;
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