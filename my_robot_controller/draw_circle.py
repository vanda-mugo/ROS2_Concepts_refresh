#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawCircle(Node):
    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Draw Circle Node has been started.')
        self.timer_ = self.create_timer(0.5, self.send_velocity_timer_callback)  # Timer to call the callback every second

    def send_velocity_timer_callback(self):
        self.get_logger().info('Drawing a circle...')
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = DrawCircle()
    rclpy.spin(node)

    rclpy.shutdown()