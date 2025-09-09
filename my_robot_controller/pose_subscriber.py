#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        # to create a subscriber
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info('Pose Subscriber Node has been started.')


    def pose_callback(self, msg: Pose):
        self.get_logger().info(f'Turtle Position - x: {msg.x}, y: {msg.y}, theta: {msg.theta}')


def main(args=None):
    rclpy.init(args=args)

    node = PoseSubscriber()
    rclpy.spin(node)

    rclpy.shutdown()