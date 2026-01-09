// Copyright (C) Aurora Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0

#ifndef NAV_SERIAL__PROTOCOL__PACKET_PROTOCOL_HPP_
#define NAV_SERIAL__PROTOCOL__PACKET_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <optional>
#include <array>
#include <functional>

#include "nav_serial/crc.hpp"

namespace nav_serial::protocol {

// 使用 crc 函数
using aurora::serial_driver::crc8_calculate;

//=============================================================================
// 数据包常量定义
//=============================================================================

constexpr uint8_t FRAME_HEADER = 0xFF;
constexpr uint8_t FRAME_TAIL = 0x0D;
constexpr size_t PACKET_SIZE = 15;

// 数据包字段偏移量
namespace offset {
  constexpr size_t HEADER = 0;
  constexpr size_t DATA1 = 1;
  constexpr size_t DATA2 = 5;
  constexpr size_t DATA3 = 9;
  constexpr size_t CRC = 13;
  constexpr size_t TAIL = 14;
}

//=============================================================================
// 通用数据包结构
//=============================================================================

#pragma pack(push, 1)

struct RawPacket {
  uint8_t header{FRAME_HEADER};
  float data1{0.0f};
  float data2{0.0f};
  float data3{0.0f};
  uint8_t crc{0};
  uint8_t tail{FRAME_TAIL};
  
  // 计算CRC
  void calculate_crc() {
    crc = crc8_calculate(reinterpret_cast<const uint8_t*>(&data1), 12);
  }
  
  // 验证CRC
  [[nodiscard]] bool verify_crc() const {
    uint8_t expected = crc8_calculate(reinterpret_cast<const uint8_t*>(&data1), 12);
    return crc == expected;
  }
  
  // 验证帧头帧尾
  [[nodiscard]] bool verify_frame() const {
    return header == FRAME_HEADER && tail == FRAME_TAIL;
  }
  
  // 完整验证
  [[nodiscard]] bool is_valid() const {
    return verify_frame() && verify_crc();
  }
};

static_assert(sizeof(RawPacket) == PACKET_SIZE, "RawPacket size must be 15 bytes");

#pragma pack(pop)

//=============================================================================
// 业务数据结构
//=============================================================================

// 速度指令 (NUC -> C_Board)
struct VelocityCommand {
  float vx{0.0f};   // X方向线速度 m/s
  float vy{0.0f};   // Y方向线速度 m/s
  float wz{0.0f};   // Z轴角速度 rad/s
  
  // 转换为原始数据包
  [[nodiscard]] RawPacket to_packet() const {
    RawPacket packet;
    packet.data1 = vx;
    packet.data2 = vy;
    packet.data3 = wz;
    packet.calculate_crc();
    return packet;
  }
  
  // 从原始数据包解析
  static VelocityCommand from_packet(const RawPacket& packet) {
    return {packet.data1, packet.data2, packet.data3};
  }
};

// 姿态数据 (C_Board -> NUC)
struct ChassisState {
  float roll{0.0f};   // 横滚角 rad
  float pitch{0.0f};  // 俯仰角 rad
  float yaw{0.0f};    // 偏航角 rad
  
  // 转换为原始数据包
  [[nodiscard]] RawPacket to_packet() const {
    RawPacket packet;
    packet.data1 = roll;
    packet.data2 = pitch;
    packet.data3 = yaw;
    packet.calculate_crc();
    return packet;
  }
  
  // 从原始数据包解析
  static ChassisState from_packet(const RawPacket& packet) {
    return {packet.data1, packet.data2, packet.data3};
  }
};

//=============================================================================
// 数据包解析结果
//=============================================================================

enum class ParseResult {
  SUCCESS,
  INVALID_HEADER,
  INVALID_TAIL,
  CRC_ERROR,
  INCOMPLETE_DATA
};

inline const char* to_string(ParseResult result) {
  switch (result) {
    case ParseResult::SUCCESS: return "SUCCESS";
    case ParseResult::INVALID_HEADER: return "INVALID_HEADER";
    case ParseResult::INVALID_TAIL: return "INVALID_TAIL";
    case ParseResult::CRC_ERROR: return "CRC_ERROR";
    case ParseResult::INCOMPLETE_DATA: return "INCOMPLETE_DATA";
    default: return "UNKNOWN";
  }
}

//=============================================================================
// 数据包解析器
//=============================================================================

class PacketParser {
public:
  using PacketCallback = std::function<void(const RawPacket&)>;
  
  explicit PacketParser(PacketCallback callback = nullptr)
    : callback_(std::move(callback)) {}
  
  void set_callback(PacketCallback callback) {
    callback_ = std::move(callback);
  }
  
  // 输入数据并解析
  ParseResult feed(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      ParseResult result = feed_byte(data[i]);
      if (result == ParseResult::SUCCESS) {
        if (callback_) {
          callback_(current_packet_);
        }
      }
    }
    return ParseResult::SUCCESS;
  }
  
  // 获取最后一个有效数据包
  [[nodiscard]] const RawPacket& last_packet() const { return current_packet_; }
  
  // 重置解析器状态
  void reset() {
    state_ = State::WAIT_HEADER;
    buffer_index_ = 0;
  }
  
  // 获取统计信息
  [[nodiscard]] size_t success_count() const { return success_count_; }
  [[nodiscard]] size_t error_count() const { return error_count_; }

private:
  enum class State { WAIT_HEADER, RECEIVING };
  
  ParseResult feed_byte(uint8_t byte) {
    switch (state_) {
      case State::WAIT_HEADER:
        if (byte == FRAME_HEADER) {
          buffer_[0] = byte;
          buffer_index_ = 1;
          state_ = State::RECEIVING;
        }
        return ParseResult::INCOMPLETE_DATA;
        
      case State::RECEIVING:
        buffer_[buffer_index_++] = byte;
        
        if (buffer_index_ >= PACKET_SIZE) {
          state_ = State::WAIT_HEADER;
          
          // 验证帧尾
          if (buffer_[PACKET_SIZE - 1] != FRAME_TAIL) {
            ++error_count_;
            return ParseResult::INVALID_TAIL;
          }
          
          // 复制到数据包结构
          std::memcpy(&current_packet_, buffer_.data(), PACKET_SIZE);
          
          // 验证CRC
          if (!current_packet_.verify_crc()) {
            ++error_count_;
            return ParseResult::CRC_ERROR;
          }
          
          ++success_count_;
          return ParseResult::SUCCESS;
        }
        return ParseResult::INCOMPLETE_DATA;
    }
    return ParseResult::INCOMPLETE_DATA;
  }
  
  State state_{State::WAIT_HEADER};
  std::array<uint8_t, PACKET_SIZE * 2> buffer_{};
  size_t buffer_index_{0};
  RawPacket current_packet_{};
  PacketCallback callback_;
  size_t success_count_{0};
  size_t error_count_{0};
};

}  // namespace nav_serial::protocol

#endif  // NAV_SERIAL__PROTOCOL__PACKET_PROTOCOL_HPP_
