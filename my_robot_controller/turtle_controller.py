#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info('Turtle Controller Node has been started.')


def main(args=None):
    rclpy.init(args = args)

    node = TurtleController()
    rclpy.spin(node)

    rclpy.shutdown()