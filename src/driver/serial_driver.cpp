// Copyright (C) Aurora Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0

#include "nav_serial/driver/serial_driver.hpp"
#include "nav_serial/uart_transporter.hpp"

#include <iostream>

namespace nav_serial::driver {

//=============================================================================
// 构造与析构
//=============================================================================

SerialDriver::SerialDriver(SerialConfig config, DriverCallbacks callbacks)
  : config_(std::move(config))
  , callbacks_(std::move(callbacks))
  , send_interval_ms_(1000 / config_.send_rate_hz)
{
  // 设置默认传输器工厂
  transporter_factory_ = [](const SerialConfig& cfg) {
    return std::make_shared<aurora::serial_driver::UartTransporter>(
      cfg.port, cfg.baud_rate, cfg.flow_ctrl, cfg.databits, cfg.stopbits, cfg.parity);
  };
  
  // 设置解析回调
  parser_.set_callback([this](const protocol::RawPacket& packet) {
    ++stats_.rx_packets;
    if (callbacks_.on_receive) {
      auto state = protocol::ChassisState::from_packet(packet);
      callbacks_.on_receive(state);
    }
  });
}

SerialDriver::~SerialDriver() {
  stop();
}

//=============================================================================
// 生命周期管理
//=============================================================================

bool SerialDriver::start() {
  if (running_.load()) {
    return true;
  }
  
  if (!config_.is_valid()) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = "Invalid configuration";
    return false;
  }
  
  running_ = true;
  
  // 连接串口
  if (!connect()) {
    // 启动重连线程
    set_state(DriverState::RECONNECTING);
  }
  
  // 启动接收线程
  recv_thread_ = std::make_unique<std::thread>(&SerialDriver::receive_loop, this);
  
  // 启动发送线程
  send_thread_ = std::make_unique<std::thread>(&SerialDriver::send_loop, this);
  
  return true;
}

void SerialDriver::stop() {
  if (!running_.load()) {
    return;
  }
  
  running_ = false;
  set_state(DriverState::STOPPED);
  
  // 等待线程结束
  if (recv_thread_ && recv_thread_->joinable()) {
    recv_thread_->join();
  }
  if (send_thread_ && send_thread_->joinable()) {
    send_thread_->join();
  }
  
  disconnect();
}

//=============================================================================
// 连接管理
//=============================================================================

bool SerialDriver::connect() {
  set_state(DriverState::CONNECTING);
  
  try {
    transporter_ = transporter_factory_(config_);
    
    if (transporter_ && transporter_->open()) {
      set_state(DriverState::CONNECTED);
      parser_.reset();
      return true;
    } else {
      std::lock_guard<std::mutex> lock(error_mutex_);
      last_error_ = transporter_ ? transporter_->errorMessage() : "Failed to create transporter";
      set_state(DriverState::DISCONNECTED);
      return false;
    }
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = std::string("Exception: ") + e.what();
    set_state(DriverState::ERROR);
    return false;
  }
}

void SerialDriver::disconnect() {
  if (transporter_) {
    transporter_->close();
    transporter_.reset();
  }
}

//=============================================================================
// 数据发送
//=============================================================================

bool SerialDriver::send(const protocol::VelocityCommand& cmd) {
  std::lock_guard<std::mutex> lock(send_mutex_);
  send_data_ = cmd;
  return true;
}

void SerialDriver::send_loop() {
  while (running_.load()) {
    // 检查连接状态
    if (state_.load() != DriverState::CONNECTED) {
      // 尝试重连
      if (state_.load() == DriverState::RECONNECTING || 
          state_.load() == DriverState::DISCONNECTED) {
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.reconnect_interval_ms));
        
        if (running_.load() && connect()) {
          ++stats_.reconnect_count;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      continue;
    }
    
    // 发送数据
    protocol::RawPacket packet;
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      packet = send_data_.to_packet();
    }
    
    if (transporter_ && transporter_->isOpen()) {
      int written = transporter_->write(&packet, sizeof(packet));
      if (written == sizeof(packet)) {
        ++stats_.tx_packets;
      } else {
        ++stats_.tx_errors;
        // 发送失败，可能需要重连
        if (callbacks_.on_error) {
          callbacks_.on_error("Write failed, disconnecting");
        }
        disconnect();
        set_state(DriverState::RECONNECTING);
      }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(send_interval_ms_.load()));
  }
}

//=============================================================================
// 数据接收
//=============================================================================

void SerialDriver::receive_loop() {
  std::array<uint8_t, 64> buffer{};
  
  while (running_.load()) {
    if (state_.load() != DriverState::CONNECTED || !transporter_ || !transporter_->isOpen()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    
    int len = transporter_->read(buffer.data(), buffer.size());
    
    if (len > 0) {
      auto result = parser_.feed(buffer.data(), len);
      if (result != protocol::ParseResult::SUCCESS && 
          result != protocol::ParseResult::INCOMPLETE_DATA) {
        ++stats_.rx_errors;
      }
    } else if (len < 0) {
      // 读取错误
      ++stats_.rx_errors;
      if (callbacks_.on_error) {
        callbacks_.on_error("Read error, disconnecting");
      }
      disconnect();
      set_state(DriverState::RECONNECTING);
    }
  }
}

//=============================================================================
// 状态管理
//=============================================================================

void SerialDriver::set_state(DriverState new_state) {
  DriverState old_state = state_.exchange(new_state);
  if (old_state != new_state && callbacks_.on_state_change) {
    callbacks_.on_state_change(old_state, new_state);
  }
}

std::string SerialDriver::last_error() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void SerialDriver::update_send_rate(int hz) {
  if (hz > 0) {
    send_interval_ms_ = 1000 / hz;
  }
}

}  // namespace nav_serial::driver
