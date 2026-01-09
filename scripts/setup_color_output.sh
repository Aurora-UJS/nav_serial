#!/bin/bash
# ROS2 彩色日志输出配置
# 将此文件 source 到 ~/.bashrc 或 ~/.zshrc 中

# 启用 ROS2 彩色日志输出
export RCUTILS_COLORIZED_OUTPUT=1

# 设置日志格式（包含时间戳、严重性等）
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# 或者使用更详细的格式
# export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}] [{function_name}]: {message}"

echo "ROS2 彩色日志输出已启用"
echo "  - RCUTILS_COLORIZED_OUTPUT=1"
echo "  - RCUTILS_CONSOLE_OUTPUT_FORMAT=$RCUTILS_CONSOLE_OUTPUT_FORMAT"
