#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A simple ROS2 node that logs a message."""
import rclpy
from rclpy.node import Node

"""
"""

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('Hello, ROS2! This is my first node.')
        self.create_timer(1.0, self.timer_callback )  # Timer to call the callback every second

    def timer_callback(self):
        self.get_logger().info('hello ROS event')


def main(args=None):
    rclpy.init(args=args)


    # this is where the node will be created  and used
    node = MyFirstNode()
    # keep the node alive until interrupted
    rclpy.spin(node)

    rclpy.shutdown()

# Entry point for the node
if __name__ == '__main__':
    main()

