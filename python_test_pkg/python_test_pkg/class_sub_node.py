#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
file    : class_sub_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 建立 ROS2 Node 節點
    • 建立 ROS2 Sublisher
    • 接收來自 "class_topic" 的訊息

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
from test_msgs_pkg.msg import Class


# ------------------------------------------------------------------------------------ #
# ClassSubscriber 建立 ROS2 Node, Subscriber
# ------------------------------------------------------------------------------------ #
class ClassSubscriber(Node):
    def __init__(self):

        # 1. 初始化 Node 節點
        super().__init__("class_sub_node")

        # 2. 初始化 Subscriber 節點 
        self.subscription = self.create_subscription(
            Class,                             
            "/class_topic",                  
            self.callback_class_message,        
            10                                   
        )

    # --------------------------------- 訊息收到後回呼 --------------------------------- #
    def callback_class_message(self, msg:Class):
        self.get_logger().info(f"Received Class ID: {msg.id}")
        for i, student in enumerate(msg.students):
            self.get_logger().info(f"  Student {i+1}: Name = {student.name}, Age = {student.age}")


# ------------------------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)                       
    node = ClassSubscriber()                  
    try:
        rclpy.spin(node)                 
    except KeyboardInterrupt:
        node.destroy_node()                   
        rclpy.shutdown()                    

if __name__ == "__main__":
    main()