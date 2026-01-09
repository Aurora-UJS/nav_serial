#!/usr/bin/env python3
"""
串口设备检测工具
用于检测系统中的串口设备并显示详细信息
"""

import os
import subprocess
import sys

def run_command(cmd):
    """运行系统命令并返回输出"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        return f"Error: {e}"

def check_serial_devices():
    """检查串口设备"""
    print("=" * 60)
    print("串口设备检测工具")
    print("=" * 60)
    print()
    
    # 检查 ttyUSB 设备
    print("1. USB转串口设备 (/dev/ttyUSB*):")
    ttyusb_devices = run_command("ls /dev/ttyUSB* 2>/dev/null")
    if ttyusb_devices:
        for device in ttyusb_devices.split('\n'):
            if device:
                print(f"   找到: {device}")
                # 显示权限
                perms = run_command(f"ls -l {device}")
                print(f"   权限: {perms}")
                print()
    else:
        print("   未找到 ttyUSB 设备")
        print()
    
    # 检查 ttyACM 设备
    print("2. ACM串口设备 (/dev/ttyACM*):")
    ttyacm_devices = run_command("ls /dev/ttyACM* 2>/dev/null")
    if ttyacm_devices:
        for device in ttyacm_devices.split('\n'):
            if device:
                print(f"   找到: {device}")
                # 显示权限
                perms = run_command(f"ls -l {device}")
                print(f"   权限: {perms}")
                print()
    else:
        print("   未找到 ttyACM 设备")
        print()
    
    # 检查用户组
    print("3. 当前用户组权限:")
    groups = run_command("groups")
    print(f"   用户组: {groups}")
    if 'dialout' in groups:
        print("   ✓ 用户已在 dialout 组中，有串口访问权限")
    else:
        print("   ✗ 用户不在 dialout 组中，需要配置权限")
        print("   解决方法: sudo usermod -a -G dialout $USER")
        print("   然后注销并重新登录")
    print()
    
    # 使用 dmesg 查看最近的串口连接信息
    print("4. 最近的串口连接日志（最后10条）:")
    dmesg_output = run_command("dmesg | grep -i 'tty\\|serial\\|usb' | tail -n 10")
    if dmesg_output:
        for line in dmesg_output.split('\n'):
            print(f"   {line}")
    else:
        print("   无相关日志")
    print()
    
    # 使用 lsusb 查看 USB 设备
    print("5. USB设备列表:")
    lsusb_output = run_command("lsusb")
    if lsusb_output:
        for line in lsusb_output.split('\n'):
            if any(keyword in line.lower() for keyword in ['serial', 'uart', 'ch340', 'cp210', 'ftdi']):
                print(f"   {line}")
    print()
    
    print("=" * 60)
    print("提示:")
    print("  - 如果看不到设备，请检查USB连接")
    print("  - 如果设备存在但无权限访问，请运行:")
    print("    sudo bash scripts/setup_serial_permission.sh")
    print("=" * 60)

if __name__ == "__main__":
    check_serial_devices()
