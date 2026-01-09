// Copyright (C) Aurora Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0

#ifndef NAV_SERIAL__ROS__SERIAL_DRIVER_NODE_HPP_
#define NAV_SERIAL__ROS__SERIAL_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "nav_serial/driver/serial_driver.hpp"

namespace nav_serial::ros {

class SerialDriverNode : public rclcpp::Node {
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SerialDriverNode() override;

private:
  //--- 初始化 ---
  void declare_parameters();
  driver::SerialConfig load_config();
  void setup_driver();
  void setup_ros_interfaces();
  
  //--- ROS 回调 ---
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameters_change(
    const std::vector<rclcpp::Parameter>& parameters);
  
  //--- 驱动回调 ---
  void on_driver_state_change(driver::DriverState old_state, driver::DriverState new_state);
  void on_driver_receive(const protocol::ChassisState& state);
  void on_driver_error(const std::string& error);
  
  //--- 服务回调 ---
  void on_reconnect_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  //--- 数据转换 ---
  sensor_msgs::msg::Imu chassis_state_to_imu(const protocol::ChassisState& state);
  
  //--- 定时器回调 ---
  void on_statistics_timer();
  void on_send_timer();
  void on_publish_timer();
  
  //--- 成员变量 ---
  std::unique_ptr<driver::SerialDriver> driver_;
  
  // 当前速度指令
  protocol::VelocityCommand current_cmd_{};
  
  // 当前云台状态
  protocol::ChassisState current_state_{};
  
  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_srv_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  // 参数回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 统计
  size_t last_tx_packets_{0};
  size_t last_rx_packets_{0};
  size_t last_tx_errors_{0};
  size_t last_rx_errors_{0};
  
  // 配置
  std::string frame_id_{"gimbal_yaw"};
};

}  // namespace nav_serial::ros

#endif  // NAV_SERIAL__ROS__SERIAL_DRIVER_NODE_HPP_
