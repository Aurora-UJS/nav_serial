#!/usr/bin/env python3
"""
模拟发送 cmd_vel 速度指令的测试脚本
用于测试串口驱动节点
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import argparse


class CmdVelPublisher(Node):
    def __init__(self, namespace: str, mode: str, rate: float):
        super().__init__('cmd_vel_publisher')
        
        topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
        self.mode = mode
        self.t = 0.0
        self.dt = 1.0 / rate
        
        self.get_logger().info(f'Publishing to: {topic}')
        self.get_logger().info(f'Mode: {mode}, Rate: {rate} Hz')
        self.get_logger().info('Press Ctrl+C to stop')
    
    def timer_callback(self):
        msg = Twist()
        
        if self.mode == 'zero':
            # 静止
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            
        elif self.mode == 'forward':
            # 前进
            msg.linear.x = 0.5
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            
        elif self.mode == 'rotate':
            # 原地旋转
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.5
            
        elif self.mode == 'circle':
            # 画圆
            msg.linear.x = 0.3
            msg.linear.y = 0.0
            msg.angular.z = 0.3
            
        elif self.mode == 'sine':
            # 正弦波运动
            msg.linear.x = 0.5 * math.sin(self.t)
            msg.linear.y = 0.5 * math.cos(self.t)
            msg.angular.z = 0.3 * math.sin(self.t * 0.5)
            
        elif self.mode == 'square':
            # 方波（周期性切换方向）
            period = 2.0  # 2秒周期
            phase = (self.t % (period * 4)) / period
            if phase < 1:
                msg.linear.x = 0.5
            elif phase < 2:
                msg.linear.y = 0.5
            elif phase < 3:
                msg.linear.x = -0.5
            else:
                msg.linear.y = -0.5
        
        self.publisher.publish(msg)
        self.t += self.dt
        
        # 每秒打印一次状态
        if int(self.t / self.dt) % int(1.0 / self.dt) == 0:
            self.get_logger().info(
                f'vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}'
            )


def main():
    parser = argparse.ArgumentParser(description='Simulate cmd_vel publisher')
    parser.add_argument(
        '-n', '--namespace',
        default='red_standard_robot1',
        help='Robot namespace (default: red_standard_robot1)'
    )
    parser.add_argument(
        '-m', '--mode',
        choices=['zero', 'forward', 'rotate', 'circle', 'sine', 'square'],
        default='forward',
        help='Motion mode (default: forward)'
    )
    parser.add_argument(
        '-r', '--rate',
        type=float,
        default=50.0,
        help='Publish rate in Hz (default: 50)'
    )
    
    args = parser.parse_args()
    
    rclpy.init()
    node = CmdVelPublisher(args.namespace, args.mode, args.rate)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 发送停止指令
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.get_logger().info('Stopped, sent zero velocity')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
