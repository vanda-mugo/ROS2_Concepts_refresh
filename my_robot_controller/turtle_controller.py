#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        # to create a subscriber that gives us the pose within the same node 
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # now to create a publisher to send commands to the cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Turtle Controller Node has been started.')
        self.previous_x = 0.0
        self.previous_y = 0.0

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

        # we need to call the service to change the pen color based on the position
        #to avoid calling the service too many times we can add a condition
        if msg.x > 5.5 and self.previous_x <= 5.5:
            # right side - blue
            self.previous_x = msg.x
            self.get_logger().info('Changing pen to blue')
            self.call_set_pen_service(0, 0, 255, 2, 0)
        elif msg.x < 5.5 and self.previous_x >= 5.5:
            # left side - red
            self.previous_x = msg.x
            self.get_logger().info('Changing pen to red')
            self.call_set_pen_service(255, 0, 0, 2, 0)
        
        self.cmd_vel_publisher.publish(twist)
    
    # we want to create a service call from here as a client to the service turtle1/set_pen
        # this service call will basically change the pen color based on the position of the turtle
        # if the turtle is on the left side of the screen we want the color to be red otherwise blue
        # so to know the parameters you need you can check the turtlesim/srv/SetPen.srv file
        # ros2 service list 
        # ros2 service type /turtle1/set_pen
        # ros2 interface show turtlesim/srv/SetPen
        """
        uint8 r
        uint8 g
        uint8 b
        uint8 width
        uint8 off
        """
    def call_set_pen_service(self, r, g, b, width, off):
        # This function can be used to call the set_pen service to change pen properties
        # create client 
        # you provide the service type and service name 
        client = self.create_client(SetPen, '/turtle1/set_pen')
        # we need to wait for the service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for set_pen service to become available...')

        # lets create a request object
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        # now we can call the service
        # call the service in an async way
        future = client.call_async(request)
        # add a callback to be called when the service call is complete
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        # this will be called when the service call is complete
        try:
            response = future.result()
            self.get_logger().info('Set pen service call was successful.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args = args)

    node = TurtleController()
    rclpy.spin(node)

    rclpy.shutdown()