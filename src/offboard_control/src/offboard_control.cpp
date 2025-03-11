#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
  OffboardControl() 
  : Node("offboard_control"), 
    target_height_(1.5),          // 1. double类型参数
    state_(ControlState::INITIALIZATION), // 2. 状态枚举
    state_entry_time_(this->now()), // 3. 时间戳
    retry_count_(0),               // 4. 重试计数器
    pose_count_(0)                 // 5. 位姿计数器
  {
    // 初始化顺序
    initialize_parameters();
    setup_publishers();
    setup_subscribers();
    setup_clients();
    setup_timer();
    enter_state(ControlState::INITIALIZATION);
  }

private:
  // ========================
  // 类型定义
  // ========================
  enum class ControlState {
    INITIALIZATION,       // 初始化参数和组件
    WAITING_SERVICES,     // 等待ROS服务可用
    INITIAL_PUBLISHING,   // 初始设定点发布
    ENABLE_OFFBOARD,      // 切换至OFFBOARD模式
    POST_OFFBOARD_DELAY,  // 模式切换后等待
    ARMING,               // 解锁无人机
    POST_ARMING_DELAY,    // 解锁后等待
    ACTIVE,               // 主动控制阶段
    ERROR_STATE           // 错误处理状态
  };

  // ========================
  // 成员变量
  // ========================
  // 配置参数
  const int MAX_RETRIES = 3;
  double target_height_;
  double target_y_ ;
  // 状态管理
  ControlState state_;
  rclcpp::Time state_entry_time_;
  int retry_count_;
  int pose_count_;
  bool vision_memory_ = false;
  bool pose_memory1_ = false;
  double pose_memory1_x_ = 0.0;
  double pose_memory1_y_ = 0.0;
  double pose_memory1_z_ = 0.0;
  // ROS 通信组件
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 消息存储
  mavros_msgs::msg::State current_state_;
  nav_msgs::msg::Odometry current_odom_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Point vision_;

  // ========================
  // 核心逻辑方法
  // ========================
  void initialize_parameters() {
    declare_parameter("target_height", 1.5);
    target_height_ = get_parameter("target_height").as_double();
    
    pose_.pose.position.x = 0.0;
    pose_.pose.position.y = target_y_;
    pose_.pose.position.z = target_height_;
  }

  void setup_publishers() {
    auto qos = rclcpp::QoS(10).reliable();
    local_pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", qos);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "mavros/setpoint_velocity/cmd_vel", qos);
  }

  void setup_subscribers() {
    auto mavros_qos = rclcpp::QoS(10)
      .best_effort()
      .durability_volatile();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
        check_connection_status();
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/global_position/local", mavros_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        // RCLCPP_DEBUG(get_logger(), "Odom update: x=%.2f", 
        //             msg->pose.pose.position.x);
      });
  
    vision_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/yellow_center_offset", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        vision_ = *msg;
        // RCLCPP_INFO(get_logger(), "Vision update: x=%.2f, y=%.2f, z=%.2f", 
        //            msg->x, msg->y, msg->z);
      });
  }

  void setup_clients() {
    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  }

  void setup_timer() {
    timer_ = create_wall_timer(20ms, [this]() { main_loop(); });
  }

  // ========================
  // 状态机核心逻辑
  // ========================
  void main_loop() {
    publish_setpoint();  // 持续发布设定点

    switch(state_) {
      case ControlState::INITIALIZATION:    handle_initialization(); break;
      case ControlState::WAITING_SERVICES:  handle_waiting_services(); break;
      case ControlState::INITIAL_PUBLISHING:handle_initial_publishing(); break;
      case ControlState::ENABLE_OFFBOARD:   handle_enable_offboard(); break;
      case ControlState::POST_OFFBOARD_DELAY:handle_post_offboard_delay(); break;
      case ControlState::ARMING:           handle_arming(); break;
      case ControlState::POST_ARMING_DELAY: handle_post_arming_delay(); break;
      case ControlState::ACTIVE:           handle_active_state(); break;
      case ControlState::ERROR_STATE:      handle_error_state(); break;
    }
  }

  // ========================
  // 状态转换方法
  // ========================
  void enter_state(ControlState new_state) {
    state_ = new_state;
    state_entry_time_ = this->now();
    retry_count_ = 0;
    RCLCPP_INFO(get_logger(), "State transition to: %s", 
               state_to_string().c_str());
  }

  std::string state_to_string() {
    switch(state_) {
      case ControlState::INITIALIZATION:     return "INITIALIZATION";
      case ControlState::WAITING_SERVICES:   return "WAITING_SERVICES";
      case ControlState::INITIAL_PUBLISHING: return "INITIAL_PUBLISHING";
      case ControlState::ENABLE_OFFBOARD:    return "ENABLE_OFFBOARD";
      case ControlState::POST_OFFBOARD_DELAY:return "POST_OFFBOARD_DELAY";
      case ControlState::ARMING:             return "ARMING";
      case ControlState::POST_ARMING_DELAY:  return "POST_ARMING_DELAY";
      case ControlState::ACTIVE:             return "ACTIVE";
      case ControlState::ERROR_STATE:        return "ERROR_STATE";
      default:                               return "UNKNOWN";
    }
  }

  // ========================
  // 具体状态处理逻辑
  // ========================
  void handle_initialization() {
    if (set_mode_client_->service_is_ready() && arming_client_->service_is_ready()) {
      enter_state(ControlState::INITIAL_PUBLISHING);
    } else {
      enter_state(ControlState::WAITING_SERVICES);
    }
  }

  void handle_waiting_services() {
    if (services_ready()) {
      enter_state(ControlState::INITIAL_PUBLISHING);
    } else if (timeout(5s)) {
      RCLCPP_ERROR(get_logger(), "Service initialization timeout");
      enter_state(ControlState::ERROR_STATE);
    }
  }

  void handle_initial_publishing() {
    if (timeout(2s)) {
      enter_state(ControlState::ENABLE_OFFBOARD);
    }
  }

  void handle_enable_offboard() {
    try_service_call<mavros_msgs::srv::SetMode>(
      set_mode_client_,
      [](auto req) { req->custom_mode = "OFFBOARD"; },
      [this](auto res) {
        if (res->mode_sent) enter_state(ControlState::POST_OFFBOARD_DELAY);
        else handle_service_failure("SetMode");
      });
  }

  void handle_post_offboard_delay() {
    if (timeout(4s)) enter_state(ControlState::ARMING);
  }

  void handle_arming() {
    try_service_call<mavros_msgs::srv::CommandBool>(
      arming_client_,
      [](auto req) { req->value = true; },
      [this](auto res) {
        if (res->success) enter_state(ControlState::POST_ARMING_DELAY);
        else handle_service_failure("Arming");
      });
  }

  void handle_post_arming_delay() {
    if (timeout(4s)) enter_state(ControlState::ACTIVE);
  }

  void handle_active_state() {
    if (pose_count_ < 300) {
      if (timeout(500ms)) pose_count_++;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Active control");
    } else {
        publish_velocity();
    }
  }

  void handle_error_state() {
    if (current_state_.connected && timeout(5s)) {
      RCLCPP_INFO(get_logger(), "Attempting recovery");
      enter_state(ControlState::INITIALIZATION);
    }
  }

  // ========================
  // 工具方法
  // ========================
  template<typename ServiceT, typename ReqModifier, typename ResHandler>
  bool try_service_call(typename rclcpp::Client<ServiceT>::SharedPtr client,
                       ReqModifier modify_req,
                       ResHandler handle_res) 
  {
    if (retry_count_ >= MAX_RETRIES) {
      RCLCPP_ERROR(get_logger(), "Max retries(%d) exceeded", MAX_RETRIES);
      return false;
    }

    if (timeout(1s * (1 << retry_count_))) {
      auto req = std::make_shared<typename ServiceT::Request>();
      modify_req(req);

      client->async_send_request(req,
        [this, handle_res](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          try {
            auto res = future.get();
            handle_res(res);
          } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Service error: %s", e.what());
            handle_service_failure(typeid(ServiceT).name());
          }
        });

      retry_count_++;
      return true;
    }
    return false;
  }

  void handle_service_failure(const std::string& service_name) {
    RCLCPP_ERROR(get_logger(), "%s failed (%d/%d)", 
                service_name.c_str(), retry_count_, MAX_RETRIES);
    if (retry_count_ >= MAX_RETRIES) {
      enter_state(ControlState::ERROR_STATE);
    }
  }

  void check_connection_status() {
    if (!current_state_.connected && state_ != ControlState::ERROR_STATE) {
      enter_state(ControlState::ERROR_STATE);
    }
  }

  bool services_ready() const {
    return set_mode_client_->service_is_ready() && 
           arming_client_->service_is_ready();
  }

  bool timeout(auto duration) const {
    return (this->now() - state_entry_time_) > duration;
  }

  void publish_setpoint() {
    if (pose_count_ < 300) {
      pose_.header.stamp = this->now();
      local_pos_pub_->publish(pose_);
    }
  }

  void publish_velocity() {
    auto now = this->now();
    double current_x = current_odom_.pose.pose.position.x;
    double current_y = current_odom_.pose.pose.position.y;
    double current_z = current_odom_.pose.pose.position.z;
    double current_w = current_odom_.pose.pose.orientation.z;
    double target_x = vision_.y;
    double target_y_ = vision_.x;
    int vision_button = vision_.z;
    double error_y = 0 - current_y;
    double error_z = target_height_ - current_z;
    double error_w = current_w;
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = "base_link";
    if( vision_button > 0 && vision_button < 10000 && vision_memory_ == false) 
    {
      twist.twist.linear.x = -target_x/2000;
      if (fabs(target_x) < 6)
      {
        pose_memory1_x_ = current_x;
        pose_memory1_y_ = current_y;
        pose_memory1_z_ = current_z;
        vision_memory_ = true;
        RCLCPP_INFO(get_logger(),"Vision memory1");
      } 
      twist.twist.linear.y = 0.8 * error_y;  // Y轴速度修正
      twist.twist.linear.z = 0.8 * error_z;  // Z轴高度修正
      twist.twist.angular.z = 0.5 * error_w; // Yaw角速度修正
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Vision 1");
    }
    else if ( vision_button > 0 && vision_button < 10000 && vision_memory_ == true)
    {
      twist.twist.linear.x = -target_x/2000;
      twist.twist.linear.y = -target_y_/2000;
      twist.twist.linear.z = 0.8 * error_z;  // Z轴高度修正
      twist.twist.angular.z = 0.5 * error_w; // Yaw角速度修正
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Vision 2");
    }
    else 
    {
      twist.twist.linear.x = 0.2;
      twist.twist.linear.y = 0.8 * error_y;  // Y轴速度修正
      twist.twist.linear.z = 0.8 * error_z;  // Z轴高度修正
      twist.twist.angular.z = 0.5 * error_w; // Yaw角速度修正
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "Vel");
    }
    cmd_vel_pub_->publish(twist);
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    // "Velocity control | X: %.2f | Y: %.2f->%.2f (err: %.2f) | Z: %.2f->%.2f (err: %.2f) | W: %.2f->%.2f (err: %.2f)",
    // twist.twist.linear.x,current_y, target_y_, error_y, current_z, target_height_, error_z, current_w, 0.0, error_w);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}