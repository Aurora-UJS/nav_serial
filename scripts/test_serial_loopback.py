#!/usr/bin/env python3
"""
测试串口通信：发送非零速度指令，观察下位机回复
用于验证下位机是否只是回环，还是真的发送陀螺仪数据
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SerialLoopbackTest(Node):
    def __init__(self):
        super().__init__('serial_loopback_test')
        
        # 创建cmd_vel发布器
        self.pub = self.create_publisher(Twist, '/red_standard_robot1/cmd_vel', 10)
        
        # 等待一会儿让系统稳定
        time.sleep(2)
        
        self.get_logger().info('开始测试...')
        self.get_logger().info('发送测试数据：vx=1.23, vy=4.56, wz=7.89')
        
        # 发送非零速度指令
        msg = Twist()
        msg.linear.x = 1.23
        msg.linear.y = 4.56
        msg.angular.z = 7.89
        
        # 持续发送10次
        for i in range(10):
            self.pub.publish(msg)
            self.get_logger().info(f'第{i+1}次发送: vx={msg.linear.x}, vy={msg.linear.y}, wz={msg.angular.z}')
            time.sleep(0.5)
        
        self.get_logger().info('测试完成！')
        self.get_logger().info('观察下位机是否回复非零数据:')
        self.get_logger().info('  - 如果回复也是1.23/4.56/7.89 → 下位机只是回环')
        self.get_logger().info('  - 如果回复还是0/0/0 → 下位机没有回环，但陀螺仪数据是0')
        self.get_logger().info('  - 如果回复是其他非零值 → 下位机正常发送陀螺仪数据')

def main():
    rclpy.init()
    node = SerialLoopbackTest()
    
    # 等待所有消息发送完成
    time.sleep(6)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
