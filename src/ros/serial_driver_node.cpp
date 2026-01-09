// Copyright (C) Aurora Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0

#include "nav_serial/ros/serial_driver_node.hpp"

namespace nav_serial::ros {

//=============================================================================
// 构造与析构
//=============================================================================

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options)
  : Node("serial_driver_node", options)
{
  RCLCPP_INFO(get_logger(), "Initializing Serial Driver Node...");
  
  declare_parameters();
  setup_driver();
  setup_ros_interfaces();
  
  RCLCPP_INFO(get_logger(), "Serial Driver Node initialized successfully");
}

SerialDriverNode::~SerialDriverNode() {
  if (driver_) {
    driver_->stop();
  }
}

//=============================================================================
// 参数声明与加载
//=============================================================================

void SerialDriverNode::declare_parameters() {
  // 串口配置
  declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  declare_parameter<int>("baud_rate", 115200);
  declare_parameter<int>("send_rate", 100);
  
  // 重连配置
  declare_parameter<int>("reconnect_interval_ms", 1000);
  declare_parameter<int>("max_reconnect_attempts", -1);
  
  // ROS 配置
  declare_parameter<std::string>("frame_id", "gimbal_yaw");
  declare_parameter<bool>("enable_statistics", true);
  declare_parameter<int>("statistics_interval_sec", 10);
}

driver::SerialConfig SerialDriverNode::load_config() {
  driver::SerialConfig config;
  
  config.port = get_parameter("serial_port").as_string();
  config.baud_rate = get_parameter("baud_rate").as_int();
  config.send_rate_hz = get_parameter("send_rate").as_int();
  config.reconnect_interval_ms = get_parameter("reconnect_interval_ms").as_int();
  config.max_reconnect_attempts = get_parameter("max_reconnect_attempts").as_int();
  
  frame_id_ = get_parameter("frame_id").as_string();
  
  return config;
}

//=============================================================================
// 驱动设置
//=============================================================================

void SerialDriverNode::setup_driver() {
  auto config = load_config();
  
  RCLCPP_INFO(get_logger(), "Serial Configuration:");
  RCLCPP_INFO(get_logger(), "  Port: %s", config.port.c_str());
  RCLCPP_INFO(get_logger(), "  Baud Rate: %d", config.baud_rate);
  RCLCPP_INFO(get_logger(), "  Send Rate: %d Hz", config.send_rate_hz);
  
  // 设置回调
  driver::DriverCallbacks callbacks;
  callbacks.on_state_change = [this](auto old_s, auto new_s) {
    on_driver_state_change(old_s, new_s);
  };
  callbacks.on_receive = [this](const auto& state) {
    on_driver_receive(state);
  };
  callbacks.on_error = [this](const auto& error) {
    on_driver_error(error);
  };
  
  // 创建驱动
  driver_ = std::make_unique<driver::SerialDriver>(config, callbacks);
  
  // 启动驱动
  if (!driver_->start()) {
    RCLCPP_ERROR(get_logger(), "Failed to start serial driver: %s", 
                 driver_->last_error().c_str());
  }
}

//=============================================================================
// ROS 接口设置
//=============================================================================

void SerialDriverNode::setup_ros_interfaces() {
  // 订阅速度指令
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&SerialDriverNode::on_cmd_vel, this, std::placeholders::_1));
  
  // 发布 IMU 数据
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "serial/gimbal_joint_state", rclcpp::QoS(10));
  
  // 重连服务
  reconnect_srv_ = create_service<std_srvs::srv::Trigger>(
    "serial/reconnect",
    std::bind(&SerialDriverNode::on_reconnect_request, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 参数动态更新
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&SerialDriverNode::on_parameters_change, this, std::placeholders::_1));
  
  // 统计定时器
  if (get_parameter("enable_statistics").as_bool()) {
    int interval = get_parameter("statistics_interval_sec").as_int();
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(interval),
      std::bind(&SerialDriverNode::on_statistics_timer, this));
  }
  
  // 发送定时器
  int send_rate = get_parameter("send_rate").as_int();
  send_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / send_rate),
    std::bind(&SerialDriverNode::on_send_timer, this));
  
  // IMU发布定时器 (与发送频率相同)
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / send_rate),
    std::bind(&SerialDriverNode::on_publish_timer, this));
  
  RCLCPP_INFO(get_logger(), "ROS Interfaces:");
  RCLCPP_INFO(get_logger(), "  Subscribe: %s", cmd_vel_sub_->get_topic_name());
  RCLCPP_INFO(get_logger(), "  Publish: %s", imu_pub_->get_topic_name());
  RCLCPP_INFO(get_logger(), "  Service: %s", reconnect_srv_->get_service_name());
}

//=============================================================================
// ROS 回调
//=============================================================================

void SerialDriverNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // 更新当前速度指令
  current_cmd_.vx = static_cast<float>(msg->linear.x);
  current_cmd_.vy = static_cast<float>(msg->linear.y);
  current_cmd_.wz = static_cast<float>(msg->angular.z);
}

rcl_interfaces::msg::SetParametersResult SerialDriverNode::on_parameters_change(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto& param : parameters) {
    if (param.get_name() == "send_rate" && driver_) {
      int rate = param.as_int();
      if (rate > 0) {
        driver_->update_send_rate(rate);
        RCLCPP_INFO(get_logger(), "Updated send rate to %d Hz", rate);
      } else {
        result.successful = false;
        result.reason = "send_rate must be positive";
      }
    } else if (param.get_name() == "frame_id") {
      frame_id_ = param.as_string();
      RCLCPP_INFO(get_logger(), "Updated frame_id to %s", frame_id_.c_str());
    }
  }
  
  return result;
}

//=============================================================================
// 驱动回调
//=============================================================================

void SerialDriverNode::on_driver_state_change(
  driver::DriverState old_state, driver::DriverState new_state)
{
  const char* old_str = driver::to_string(old_state);
  const char* new_str = driver::to_string(new_state);
  
  switch (new_state) {
    case driver::DriverState::CONNECTED:
      RCLCPP_INFO(get_logger(), "Serial port connected [%s -> %s]", old_str, new_str);
      break;
    case driver::DriverState::DISCONNECTED:
      RCLCPP_WARN(get_logger(), "Serial port disconnected [%s -> %s]", old_str, new_str);
      break;
    case driver::DriverState::RECONNECTING:
      RCLCPP_WARN(get_logger(), "Attempting to reconnect [%s -> %s]", old_str, new_str);
      break;
    case driver::DriverState::ERROR:
      RCLCPP_ERROR(get_logger(), "Serial driver error [%s -> %s]", old_str, new_str);
      break;
    default:
      RCLCPP_INFO(get_logger(), "State change: %s -> %s", old_str, new_str);
  }
}

void SerialDriverNode::on_driver_receive(const protocol::ChassisState& state) {
  // 更新当前云台状态
  current_state_ = state;
  RCLCPP_INFO(get_logger(), "RX: roll=%.2f, pitch=%.2f, yaw=%.2f", 
              state.roll, state.pitch, state.yaw);
}

void SerialDriverNode::on_driver_error(const std::string& error) {
  RCLCPP_ERROR(get_logger(), "Serial driver error: %s", error.c_str());
}

//=============================================================================
// 服务回调
//=============================================================================

void SerialDriverNode::on_reconnect_request(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Reconnect service called");
  
  if (driver_) {
    driver_->stop();
    
    // 重新加载配置
    auto config = load_config();
    driver::DriverCallbacks callbacks;
    callbacks.on_state_change = [this](auto old_s, auto new_s) {
      on_driver_state_change(old_s, new_s);
    };
    callbacks.on_receive = [this](const auto& state) {
      on_driver_receive(state);
    };
    callbacks.on_error = [this](const auto& error) {
      on_driver_error(error);
    };
    
    driver_ = std::make_unique<driver::SerialDriver>(config, callbacks);
    
    if (driver_->start()) {
      response->success = true;
      response->message = "Reconnect initiated successfully";
    } else {
      response->success = false;
      response->message = driver_->last_error();
    }
  } else {
    response->success = false;
    response->message = "Driver not initialized";
  }
}

//=============================================================================
// 数据转换
//=============================================================================

sensor_msgs::msg::Imu SerialDriverNode::chassis_state_to_imu(
  const protocol::ChassisState& state)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  
  // 欧拉角转四元数 (ZYX顺序)
  double cy = std::cos(state.yaw * 0.5);
  double sy = std::sin(state.yaw * 0.5);
  double cp = std::cos(state.pitch * 0.5);
  double sp = std::sin(state.pitch * 0.5);
  double cr = std::cos(state.roll * 0.5);
  double sr = std::sin(state.roll * 0.5);
  
  msg.orientation.w = cr * cp * cy + sr * sp * sy;
  msg.orientation.x = sr * cp * cy - cr * sp * sy;
  msg.orientation.y = cr * sp * cy + sr * cp * sy;
  msg.orientation.z = cr * cp * sy - sr * sp * cy;
  
  // 协方差设置为-1表示未知
  msg.orientation_covariance[0] = -1;
  msg.angular_velocity_covariance[0] = -1;
  msg.linear_acceleration_covariance[0] = -1;
  
  return msg;
}

//=============================================================================
// 定时器回调
//=============================================================================

void SerialDriverNode::on_send_timer() {
  if (driver_) {
    driver_->send(current_cmd_);
    RCLCPP_INFO(get_logger(), "TX: vx=%.2f, vy=%.2f, wz=%.2f", 
                current_cmd_.vx, current_cmd_.vy, current_cmd_.wz);
  }
}

void SerialDriverNode::on_publish_timer() {
  // 定时发布当前云台状态
  auto imu_msg = chassis_state_to_imu(current_state_);
  imu_pub_->publish(imu_msg);
}

void SerialDriverNode::on_statistics_timer() {
  if (!driver_) return;
  
  const auto& stats = driver_->statistics();
  auto state = driver_->state();
  
  // 计算速率
  size_t current_tx = stats.tx_packets.load();
  size_t current_rx = stats.rx_packets.load();
  size_t current_tx_err = stats.tx_errors.load();
  size_t current_rx_err = stats.rx_errors.load();
  
  double tx_rate = static_cast<double>(current_tx - last_tx_packets_) / 
                   get_parameter("statistics_interval_sec").as_int();
  double rx_rate = static_cast<double>(current_rx - last_rx_packets_) / 
                   get_parameter("statistics_interval_sec").as_int();
  double tx_err_rate = static_cast<double>(current_tx_err - last_tx_errors_) / 
                       get_parameter("statistics_interval_sec").as_int();
  double rx_err_rate = static_cast<double>(current_rx_err - last_rx_errors_) / 
                       get_parameter("statistics_interval_sec").as_int();
  
  RCLCPP_INFO(get_logger(), 
    "📊 Statistics: State=%s, TX=%.1f/s (total:%lu), RX=%.1f/s (total:%lu), "
    "TX_Err=%.1f/s, RX_Err=%.1f/s, Reconnects=%lu",
    driver::to_string(state),
    tx_rate, current_tx,
    rx_rate, current_rx,
    tx_err_rate,
    rx_err_rate,
    stats.reconnect_count.load());
  
  // 保存当前值
  last_tx_packets_ = current_tx;
  last_rx_packets_ = current_rx;
  last_tx_errors_ = current_tx_err;
  last_rx_errors_ = current_rx_err;
}

}  // namespace nav_serial::ros

//=============================================================================
// 主函数
//=============================================================================

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<nav_serial::ros::SerialDriverNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
