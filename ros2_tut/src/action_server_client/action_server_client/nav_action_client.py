#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from custom_interfaces.action import Robot2dNav  # Adjust import based on your package structure

class NavActionClient(Node):
    def __init__(self):
        super().__init__('nav_action_client')
        # Create an action client
        self._action_client = ActionClient(self, Robot2dNav, '/navigate_to_point')
        self.declare_parameter('goal_x',8.0)
        self.declare_parameter('goal_y',8.0)
        self.declare_parameter('agent_name','turtle1')
        self.get_logger().info("Waiting for action server to start...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server started, ready to send goals.")


    def send_goal(self):
        # Create a goal to send to the action server
        goal_x=float(self.get_parameter('goal_x').value)
        goal_y=float(self.get_parameter('goal_y').value)
        agent_name=self.get_parameter('agent_name').get_parameter_value().string_value # Example agent name
        goal_msg = Robot2dNav.Goal()
        goal_msg.pose.x = goal_x
        goal_msg.pose.y = goal_y
        goal_msg.agent_name = agent_name
        self.get_logger().info(f"Sending goal to {agent_name} at ({goal_x}, {goal_y})")
        self._action_client.wait_for_server()

        self.goal_future= self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)

        self.goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal was rejected by the server.")
            return

        self.get_logger().info("Goal accepted by the server.")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Goal reached successfully!")
        else:
            self.get_logger().info("Failed to reach the goal.")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance to goal: {feedback.distance_to_goal}")

def main(args=None):
    rclpy.init(args=args)
    try:
        client = NavActionClient()
        client.send_goal()  # Example goal
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()