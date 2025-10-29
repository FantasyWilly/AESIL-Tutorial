#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

"""
file    : student_pub_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 建立 ROS2 Node 節點
    • 建立 ROS2 Publisher
    • 發布 自定義消息(Student) 至 Topic

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

# 專案內部模組庫
from test_msgs_pkg.msg import Student


# ------------------------------------------------------------------------------------ #
# StudentPubNode 建立 ROS2 Node, Publisher
# ------------------------------------------------------------------------------------ #
class StudentPubNode(Node):
    def __init__(self):

        # 1. 初始化 Node 節點
        super().__init__('student_pub_node')

        # 2. 初始化 Publisher，發布到 '/student_topic'
        self.publisher = self.create_publisher(Student, '/student_topic', 10)

        # 3. 初始化計數器與定時器 (1Hz)
        self.counter = 1
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('StudentPubNode 已啟動，開始發布 Student 訊息')

    # ------------------------------ 1Hz 發布 Student 資料 ------------------------------ #
    def timer_callback(self):

        # 1. 建立 Student 訊息
        msg = Student()
        msg.age = 16
        msg.name = f"number - {self.counter}"

        # 2. 發布並記錄
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published Student: {msg.name}, age={msg.age}"
        )

        # 3. 計數器 +1
        self.counter += 1


# ------------------------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = StudentPubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
