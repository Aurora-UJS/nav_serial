#!/bin/bash

# 串口权限配置脚本
# 用于配置串口设备权限，使普通用户可以访问串口设备

echo "===== 串口权限配置工具 ====="
echo ""

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then 
    echo "请使用 sudo 运行此脚本"
    echo "用法: sudo bash setup_serial_permission.sh"
    exit 1
fi

echo "1. 添加当前用户到 dialout 组..."
ACTUAL_USER=${SUDO_USER:-$USER}
usermod -a -G dialout $ACTUAL_USER
echo "   用户 $ACTUAL_USER 已添加到 dialout 组"

echo ""
echo "2. 创建 udev 规则..."
cat > /etc/udev/rules.d/99-serial.rules << 'EOF'
# USB转串口设备权限配置
KERNEL=="ttyUSB*", MODE="0666"
KERNEL=="ttyACM*", MODE="0666"

# 根据设备ID固定设备名（可选，根据实际情况修改）
# 查看设备ID: udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
# SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="c_board", MODE="0666"
EOF

echo "   udev 规则已创建: /etc/udev/rules.d/99-serial.rules"

echo ""
echo "3. 重新加载 udev 规则..."
udevadm control --reload-rules
udevadm trigger

echo ""
echo "===== 配置完成 ====="
echo ""
echo "重要提示："
echo "1. 请注销并重新登录，或重启系统使 dialout 组权限生效"
echo "2. 之后您可以无需 sudo 直接访问串口设备"
echo ""
echo "验证方法："
echo "  groups \$USER                     # 检查是否包含 dialout 组"
echo "  ls -l /dev/ttyUSB0              # 检查设备权限（应该是 crw-rw-rw-）"
echo ""
