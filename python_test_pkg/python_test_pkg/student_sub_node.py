#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
file    : student_sub_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 建立 ROS2 Node 節點
    • 建立 ROS2 Sublisher
    • 接收來自 "student_topic" 的訊息

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
# StudentSubscriber 建立 ROS2 Node, Subscriber
# ------------------------------------------------------------------------------------ #
class StudentSubscriber(Node):
    def __init__(self):

        # 1. 初始化 Node 節點
        super().__init__("student_sub_node")   

        # 2. 初始化 Subscriber 節點 
        self.subscription = self.create_subscription(
            Student,                            
            "/student_topic",                    
            self.callback_student_message,     
            10                                
        )

    # --------------------------------- 訊息收到後回呼 --------------------------------- #
    def callback_student_message(self, msg:Student):
        self.get_logger().info(f"Received: Name: {msg.name}, Age: {msg.age}")


# ------------------------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)                       
    node = StudentSubscriber()                
    try:
        rclpy.spin(node)                  
    except KeyboardInterrupt:
        node.destroy_node()                    
        rclpy.shutdown()                       

if __name__ == "__main__":
    main()