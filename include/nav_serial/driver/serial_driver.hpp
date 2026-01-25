// Copyright (C) Aurora Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0

#ifndef NAV_SERIAL__DRIVER__SERIAL_DRIVER_HPP_
#define NAV_SERIAL__DRIVER__SERIAL_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "nav_serial/protocol/packet_protocol.hpp"
#include "nav_serial/transporter_interface.hpp"

namespace nav_serial::driver {

//=============================================================================
// 配置结构
//=============================================================================

struct SerialConfig {
  std::string port{"/dev/ttyUSB0"};
  int baud_rate{115200};
  int flow_ctrl{0};
  int databits{8};
  int stopbits{1};
  char parity{'N'};
  
  // 超时和重试配置
  int reconnect_interval_ms{1000};
  int max_reconnect_attempts{-1};  // -1 表示无限重试
  
  // 发送配置
  int send_rate_hz{100};
  
  // 验证配置有效性
  [[nodiscard]] bool is_valid() const {
    return !port.empty() && 
           baud_rate > 0 && 
           send_rate_hz > 0 &&
           (databits >= 5 && databits <= 8);
  }
};

//=============================================================================
// 驱动状态
//=============================================================================

enum class DriverState {
  DISCONNECTED,
  CONNECTING,
  CONNECTED,
  RECONNECTING,
  ERROR,
  STOPPED
};

inline const char* to_string(DriverState state) {
  switch (state) {
    case DriverState::DISCONNECTED: return "DISCONNECTED";
    case DriverState::CONNECTING: return "CONNECTING";
    case DriverState::CONNECTED: return "CONNECTED";
    case DriverState::RECONNECTING: return "RECONNECTING";
    case DriverState::ERROR: return "ERROR";
    case DriverState::STOPPED: return "STOPPED";
    default: return "UNKNOWN";
  }
}

//=============================================================================
// 统计信息
//=============================================================================

struct DriverStatistics {
  std::atomic<uint64_t> tx_packets{0};
  std::atomic<uint64_t> rx_packets{0};
  std::atomic<uint64_t> tx_errors{0};
  std::atomic<uint64_t> rx_errors{0};
  std::atomic<uint64_t> reconnect_count{0};
  
  void reset() {
    tx_packets = 0;
    rx_packets = 0;
    tx_errors = 0;
    rx_errors = 0;
    reconnect_count = 0;
  }
};

//=============================================================================
// 回调接口
//=============================================================================

struct DriverCallbacks {
  using StateCallback = std::function<void(DriverState old_state, DriverState new_state)>;
  using ReceiveCallback = std::function<void(const protocol::ChassisState&)>;
  using ErrorCallback = std::function<void(const std::string& error)>;
  
  StateCallback on_state_change;
  ReceiveCallback on_receive;
  ErrorCallback on_error;
};

//=============================================================================
// 串口驱动类
//=============================================================================

class SerialDriver {
public:
  using TransporterFactory = std::function<std::shared_ptr<aurora::serial_driver::TransporterInterface>(const SerialConfig&)>;
  
  explicit SerialDriver(SerialConfig config, DriverCallbacks callbacks = {});
  ~SerialDriver();
  
  // 禁止拷贝
  SerialDriver(const SerialDriver&) = delete;
  SerialDriver& operator=(const SerialDriver&) = delete;
  
  // 允许移动
  SerialDriver(SerialDriver&&) noexcept = default;
  SerialDriver& operator=(SerialDriver&&) noexcept = default;
  
  //--- 生命周期管理 ---
  bool start();
  void stop();
  [[nodiscard]] bool is_running() const { return running_.load(); }
  
  //--- 数据发送 ---
  bool send(const protocol::VelocityCommand& cmd);
  
  //--- 状态查询 ---
  [[nodiscard]] DriverState state() const { return state_.load(); }
  [[nodiscard]] const SerialConfig& config() const { return config_; }
  [[nodiscard]] const DriverStatistics& statistics() const { return stats_; }
  [[nodiscard]] std::string last_error() const;
  
  //--- 配置更新（运行时） ---
  void update_send_rate(int hz);
  
  //--- 设置传输器工厂（用于测试/依赖注入） ---
  void set_transporter_factory(TransporterFactory factory) {
    transporter_factory_ = std::move(factory);
  }

private:
  void set_state(DriverState new_state);
  bool connect();
  void disconnect();
  void receive_loop();
  void send_loop();
  
  // 配置
  SerialConfig config_;
  DriverCallbacks callbacks_;
  
  // 传输层
  std::shared_ptr<aurora::serial_driver::TransporterInterface> transporter_;
  TransporterFactory transporter_factory_;
  
  // 协议解析
  protocol::PacketParser parser_;
  
  // 状态
  std::atomic<DriverState> state_{DriverState::DISCONNECTED};
  std::atomic<bool> running_{false};
  
  // 发送数据
  protocol::VelocityCommand send_data_;
  std::mutex send_mutex_;
  std::atomic<int> send_interval_ms_{10};
  
  // 线程
  std::unique_ptr<std::thread> recv_thread_;
  std::unique_ptr<std::thread> send_thread_;
  
  // 统计
  DriverStatistics stats_;
  
  // 错误信息
  mutable std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace nav_serial::driver

#endif  // NAV_SERIAL__DRIVER__SERIAL_DRIVER_HPP_
