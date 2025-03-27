#!/usr/bin/env python3

import rospy
import actionlib
from action_server_client.msg import Robot2dNavAction, Robot2dNavGoal  

class NavActionClient:
    def __init__(self):
        rospy.init_node('nav_action_client')

        # Create an action client
        self.client = actionlib.SimpleActionClient('navigate_to_point', Robot2dNavAction)

        rospy.loginfo("Waiting for action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Action server started, ready to send goals.")

    def send_goal(self, goal_x, goal_y, namespace):
        # Create a goal to send to the action server
        goal = Robot2dNavGoal()
        goal.pose.x = goal_x
        goal.pose.y = goal_y
        goal.namespace = namespace

        rospy.loginfo(f"Sending goal: x={goal_x}, y={goal_y}, namespace={namespace}")
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        # Wait for the result
        self.client.wait_for_result()
        result = self.client.get_result()

        if result.success:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.loginfo("Failed to reach the goal.")

    def feedback_callback(self, feedback):
        rospy.loginfo(f"Distance to goal: {feedback.distance_to_goal}")

if __name__ == '__main__':
    try:
        client = NavActionClient()
        client.send_goal(goal_x=7.0, goal_y=7.0, namespace="turtle1")  # Example goal
    except rospy.ROSInterruptException:
        pass