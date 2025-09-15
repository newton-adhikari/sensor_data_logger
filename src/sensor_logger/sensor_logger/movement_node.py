#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovementPublisher(Node):
    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_command)
        self.get_logger().info('MovementPublisher started:::')

    def publish_command(self):
        msg = Twist()
        msg.linear.x = 0.2  # Move forward
        msg.angular.z = 0.5  # Turn slightly
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = MovementPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
