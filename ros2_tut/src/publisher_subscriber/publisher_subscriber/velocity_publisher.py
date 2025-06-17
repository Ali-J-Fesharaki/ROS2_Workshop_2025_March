#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.declare_parameter('frequency', 2.0)
        frequency=self.get_parameter('frequency').get_parameter_value().double_value  # Declare a parameter for frequency
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(frequency, self.publish_velocity)  # Timer with 2 Hz frequency
        self.vel_msg = Twist()
        self.get_logger().info("Velocity Publisher Node has been started.")

    def publish_velocity(self):
        connections = self.vel_publisher.get_subscription_count()
        if connections > 0:
            self.vel_msg.linear.x = 1.0  # Move forward
            self.vel_msg.angular.z = 0.5  # Rotate
            self.vel_publisher.publish(self.vel_msg)
            self.get_logger().info(f"Published velocity: linear.x={self.vel_msg.linear.x}, angular.z={self.vel_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        velocity_publisher.get_logger().info("Node stopped by user.")
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
