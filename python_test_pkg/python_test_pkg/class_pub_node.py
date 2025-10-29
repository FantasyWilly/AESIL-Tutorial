#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
file    : class_pub_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 建立 ROS2 Node 節點
    • 建立 ROS2 Publisher
    • 發布 自定義消息(Class, Student) 至 Topic

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
from test_msgs_pkg.msg import Class, Student


# ------------------------------------------------------------------------------------ #
# ClassPubNode 建立 ROS2 Node, Publisher
# ------------------------------------------------------------------------------------ #
class ClassPubNode(Node):
    def __init__(self):

        # 1. 初始化 Node 節點
        super().__init__('class_pub_node')

        # 2. 初始化 Publisher，發布到 '/class_topic'
        self.publisher = self.create_publisher(Class, '/class_topic', 10)

        # 3. 初始化定時器與計數器
        self.counter = 1
        self.timer = self.create_timer(1.0, self.publish_class_message)

        self.get_logger().info('ClassPubNode 已啟動，開始發布 Class 訊息')

    # ------------------------------ 1Hz 發布 Class 資料 ------------------------------ #
    def publish_class_message(self):
        # 1. 建立 Class 訊息並設定 ID
        class_msg = Class()
        class_msg.id = self.counter

        # 2. 建立 Student 訊息並加入 class_msg
        student_a = Student()
        student_a.name = 'Alice'
        student_a.age = 15

        student_b = Student()
        student_b.name = 'Bob'
        student_b.age = 16

        class_msg.students = [student_a, student_b]

        # 3. 發布消息並記錄
        self.publisher.publish(class_msg)
        student_info = ', '.join(f"{s.name}({s.age})" for s in class_msg.students)
        self.get_logger().info(
            f'Published Class id:{class_msg.id} - students: {student_info}'
        )

        # 4. 計數器遞增
        self.counter += 1


# ------------------------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = ClassPubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
