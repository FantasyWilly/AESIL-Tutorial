#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
file    : hello_sub_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 建立 ROS2 Node 節點
    • 建立 ROS2 Subscriber
    • 接收來自 "hello_topic" 的訊息

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
"""

# ------------------------------------------------------------------------------------ #
# Import
# ------------------------------------------------------------------------------------ #
# ROS 2 Python API
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ------------------------------------------------------------------------------------ #
# HelloSubNode 建立 ROS2 Node, Subscriber
# ------------------------------------------------------------------------------------ #
class HelloSubNode(Node):
    def __init__(self):

        # 1. 初始化 Node 節點
        super().__init__('hello_sub_node')

        # 2. 初始化 Subscriber 節點
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscriber 節點已啟動，等待接收訊息...')

    # --------------------------------- 訊息收到後回呼 --------------------------------- #
    def listener_callback(self, msg: String):

        # 1. 取得訊息內容
        data = msg.data
        
        # 2. 印出至 console
        self.get_logger().info(f"Received message: '{data}'")


# ------------------------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = HelloSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()