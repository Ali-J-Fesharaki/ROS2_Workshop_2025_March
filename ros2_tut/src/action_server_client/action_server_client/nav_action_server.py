#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.action import ActionServer
from custom_interfaces.action import Robot2dNav
import math
import time


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
        self.timer = None
        self.operation_finished= False
        self.feedback_msg = Robot2dNav.Feedback()
        self.result = Robot2dNav.Result()
        self.result.success = False
        
    def pose_callback(self, msg):
        # Update the current pose of the robot
        self.current_pose = {
            'x': msg.x,
            'y': msg.y,
            'yaw': msg.theta
        }

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: x={goal_handle.request.pose.x}, y={goal_handle.request.pose.y}, agent_name={goal_handle.request.agent_name}")
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.operation_finished=False
        self.result.success=False
        # Update topic names based on the agent_name
        self.current_pose = {
            'x': 0,
            'y': 0,
            'yaw': 0
        }
        cmd_vel_topic = f"/{goal_handle.request.agent_name}/cmd_vel"
        pose_topic = f"/{goal_handle.request.agent_name}/pose"

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pose_subscription = self.create_subscription(Pose, pose_topic, self.pose_callback, 10)
        self.goal_handle = goal_handle
        while(self.operation_finished == False):
            time.sleep(0.1)
        print("Operation finished")
        self.goal_handle.succeed()
        return self.result
        # Start a timer to control the robot
        

    def control_loop(self):
        if self.current_pose is None:
            return

        # Calculate errors
        x_error = self.goal_handle.request.pose.x - self.current_pose['x']
        y_error = self.goal_handle.request.pose.y - self.current_pose['y']
        self.get_logger().info(f"Current Pose: {self.current_pose}, Goal Pose: {self.goal_handle.request.pose}")
        distance_to_goal = math.sqrt(x_error**2 + y_error**2)
        goal_angle = math.atan2(y_error, x_error)
        yaw_error = goal_angle - self.current_pose['yaw']
        self.get_logger().info(f"current_angle: {self.current_pose['yaw']}, goal_angle: {goal_angle}, yaw_error: {yaw_error}")

        # Normalize yaw error
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # Publish feedback
        self.feedback_msg.distance_to_goal = distance_to_goal
        self.goal_handle.publish_feedback(self.feedback_msg)

        # Check if goal is reached
        if distance_to_goal < 0.1:  # Threshold for reaching the goal
            self.get_logger().info("Goal reached!")
            self.result.success = True
            self.cmd_vel_publisher.publish(Twist())  # Stop the robot
            self.operation_finished = True
            self.timer.cancel()  # Stop the timer
            self.pose_subscription.destroy()
            return 

        # Calculate control commands
        cmd_vel = Twist()
        if abs(yaw_error) > 0.05:  # Threshold for yaw correction
            cmd_vel.angular.z = 1.0 if yaw_error > 0 else -1.0
        else:
            cmd_vel.angular.z = 0.0

        if distance_to_goal > 0.1:  # Threshold for forward motion
            cmd_vel.linear.x = 1.0
        else:
            cmd_vel.linear.x = 0.0

        # Publish cmd_vel
        self.cmd_vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    server = NavActionServer()
    try:
        executor=rclpy.executors.MultiThreadedExecutor()
        executor.add_node(server)
        executor.spin()
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down...')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
