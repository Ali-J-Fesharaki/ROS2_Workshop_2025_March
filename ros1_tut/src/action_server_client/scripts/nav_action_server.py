#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from actionlib import SimpleActionServer
from action_server_client.msg import Robot2dNavAction, Robot2dNavFeedback, Robot2dNavResult  


class NavActionServer:
    def __init__(self):
        rospy.init_node('nav_action_server')

        # Action server setup
        self.action_server = SimpleActionServer(
            'navigate_to_point',
            Robot2dNavAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.action_server.start()

        self.current_pose = None

    def pose_callback(self, msg):
        # Update the current pose of the robot
        self.current_pose = {
            'x': msg.x,
            'y': msg.y,
            'yaw': msg.theta
        }

    def execute_callback(self, goal):
        self.cmd_vel_publisher=None # Clear previous publisher if exists
        self.pose_subscriber=None # Clear previous subscriber if exists
        x_error, y_error, distance_to_goal, goal_angle, yaw_error=(None,None,None,None,None) # Clear previous variables
        self.current_pose = {
            'x': 0,
            'y': 0,
            'yaw':0
        }
        rospy.loginfo(f"Received goal: x={goal.pose.x}, y={goal.pose.y}, agent_name={goal.agent_name}")

        # Update topic names based on the agent_name
        cmd_vel_topic = f"/{goal.agent_name}/cmd_vel"
        pose_topic = f"/{goal.agent_name}/pose"

        # Publishers and subscribers
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber(pose_topic, Pose, self.pose_callback)

        feedback = Robot2dNavFeedback()
        result = Robot2dNavResult()

        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not result.success:
            if self.current_pose is None:
                continue

            # Calculate errors
            x_error = goal.pose.x - self.current_pose['x']
            y_error = goal.pose.y - self.current_pose['y']
            rospy.loginfo(f"Current Pose: {self.current_pose}, Goal Pose: {goal.pose}")
            distance_to_goal = math.sqrt(x_error**2 + y_error**2)
            goal_angle = math.atan2(y_error, x_error)
            yaw_error = goal_angle - self.current_pose['yaw']
            rospy.loginfo(f"current_angle: {self.current_pose['yaw']}, goal_angle: {goal_angle}, yaw_error: {yaw_error}")

            # Normalize yaw error
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            # Publish feedback
            feedback.distance_to_goal = distance_to_goal
            self.action_server.publish_feedback(feedback)

            # Check if goal is reached
            if distance_to_goal < 0.1:  # Threshold for reaching the goal
                rospy.loginfo("Goal reached!")
                result.success = True
                self.cmd_vel_publisher.publish(Twist())  # Stop the robot
                self.action_server.set_succeeded(result)
                self.pose_subscriber.unregister()
                del self.pose_subscriber , self.cmd_vel_publisher
                return

            # Calculate control commands
            cmd_vel = Twist()
            if abs(yaw_error) > 0.05:  # Threshold for yaw correction
                cmd_vel.angular.z = 1 if yaw_error > 0 else -1
            else:
                cmd_vel.angular.z = 0.0

            if distance_to_goal > 0.1:  # Threshold for forward motion
                cmd_vel.linear.x = 2
            else:
                cmd_vel.linear.x = 0.0

            # Publish cmd_vel
            self.cmd_vel_publisher.publish(cmd_vel)

            rate.sleep()


if __name__ == '__main__':
    try:
        server = NavActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
