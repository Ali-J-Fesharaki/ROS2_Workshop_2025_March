#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Position Subscriber Node has been started.")

    def pose_callback(self, pos_msg):
        self.get_logger().info(
            f"Turtle Position -> x: {pos_msg.x:.2f}, y: {pos_msg.y:.2f}, theta: {pos_msg.theta:.2f}, "
            f"linear_velocity: {pos_msg.linear_velocity:.2f}, angular_velocity: {pos_msg.angular_velocity:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PositionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
