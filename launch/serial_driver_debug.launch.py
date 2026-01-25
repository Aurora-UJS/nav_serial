#!/usr/bin/env python3
"""
串口驱动启动文件 - DEBUG模式
显示详细的RX/TX通信日志
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='red_standard_robot1',
        description='Robot namespace'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav_serial'),
            'config',
            'serial_driver.yaml'
        ]),
        description='Path to serial driver config file'
    )
    
    # 串口驱动节点 - DEBUG模式
    serial_driver_node = Node(
        package='nav_serial',
        executable='serial_driver_node',
        name='serial_driver_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],  # DEBUG日志级别
        emulate_tty=True,
    )
    
    return LaunchDescription([
        namespace_arg,
        config_file_arg,
        serial_driver_node,
    ])
