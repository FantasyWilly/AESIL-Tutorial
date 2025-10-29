#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
file    : hello_pub_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 建立 ROS2 Node 節點
    • 建立 ROS2 Publisher
    • 發布 "Hello, ROS2!" 至 Topic

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
# HelloNode 建立 ROS2 Node, Publisher
# ------------------------------------------------------------------------------------ #
class HelloNode(Node):
    def __init__(self):

        # 1. 初始化 Node 節點
        super().__init__('hello_pub_node')

        # 2. 初始化 Publisher 節點
        self.publisher = self.create_publisher(String, 'hello_topic', 10)

        # 3. 呼叫函數 (1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Node 節點創建成功, 開始將資料發布至 ROS2 Topic')
     
    # ------------------------------ 1Hz 送出 data 資料 ------------------------------- #
    def timer_callback(self):

        # 1. 定義資料格式
        msg = String()

        # 2. 編寫內容
        msg.data = 'Hello, ROS2!'

        # 3. 將訊息打包至 Publisher
        self.publisher.publish(msg)


# ------------------------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
