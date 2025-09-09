#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        # to create a subscriber that gives us the pose within the same node 
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # now to create a publisher to send commands to the cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Turtle Controller Node has been started.')

    def pose_callback(self, msg: Pose):
        # in this we can access the pose of the turtlesim and also send velocity commands based on the pose 
        self.get_logger().info(f'Turtle Position - x: {msg.x}, y: {msg.y}, theta: {msg.theta}')
        # we have a safe square of 1.0 to 10.0 in the turtlesim window
        twist = Twist()
        if msg.x >= 9.0 or msg.x <= 2.0 or msg.y >= 9.0 or msg.y <= 2.0:
            twist.linear.x = 1.0
            twist.angular.z = 0.9  # turn left
        else:
            twist.linear.x = 5.0
            twist.angular.z = 0.0  # go straight
        
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args = args)

    node = TurtleController()
    rclpy.spin(node)

    rclpy.shutdown()