#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.action import ActionServer
from action_server_client.action import Robot2dNav
import math


class NavActionServer(Node):
    def __init__(self):
        super().__init__('nav_action_server')

        # Action server setup
        self.action_server = ActionServer(
            self,
            Robot2dNav,
            'navigate_to_point',
            self.execute_callback
        )

        self.current_pose = None
        self.cmd_vel_publisher = None
        self.pose_subscription = None

    def pose_callback(self, msg):
        # Update the current pose of the robot
        self.current_pose = {
            'x': msg.x,
            'y': msg.y,
            'yaw': msg.theta
        }

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: x={goal_handle.request.pose.x}, y={goal_handle.request.pose.y}, namespace={goal_handle.request.namespace}")

        # Update topic names based on the namespace
        cmd_vel_topic = f"/{goal_handle.request.namespace}/cmd_vel"
        pose_topic = f"/{goal_handle.request.namespace}/pose"

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pose_subscription = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)

        feedback_msg = Robot2dNav.Feedback()
        result = Robot2dNav.Result()

        rate = self.create_rate(10)  # 10 Hz loop rate
        while rclpy.ok():
            if self.current_pose is None:
                continue

            # Calculate errors
            x_error = goal_handle.request.pose.x - self.current_pose['x']
            y_error = goal_handle.request.pose.y - self.current_pose['y']
            self.get_logger().info(f"Current Pose: {self.current_pose}, Goal Pose: {goal_handle.request.pose}")
            distance_to_goal = math.sqrt(x_error**2 + y_error**2)
            goal_angle = math.atan2(y_error, x_error)
            yaw_error = goal_angle - self.current_pose['yaw']
            self.get_logger().info(f"current_angle: {self.current_pose['yaw']}, goal_angle: {goal_angle}, yaw_error: {yaw_error}")

            # Normalize yaw error
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            # Publish feedback
            feedback_msg.distance_to_goal = distance_to_goal
            goal_handle.publish_feedback(feedback_msg)

            # Check if goal is reached
            if distance_to_goal < 0.1:  # Threshold for reaching the goal
                self.get_logger().info("Goal reached!")
                result.success = True
                self.cmd_vel_publisher.publish(Twist())  # Stop the robot
                goal_handle.succeed()
                return result

            # Calculate control commands
            cmd_vel = Twist()
            if abs(yaw_error) > 0.1:  # Threshold for yaw correction
                cmd_vel.angular.z = 0.5 if yaw_error > 0 else -0.5
            else:
                cmd_vel.angular.z = 0.0

            if distance_to_goal > 0.1:  # Threshold for forward motion
                cmd_vel.linear.x = 0.2
            else:
                cmd_vel.linear.x = 0.0

            # Publish cmd_vel
            self.cmd_vel_publisher.publish(cmd_vel)

            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    server = NavActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down...')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
