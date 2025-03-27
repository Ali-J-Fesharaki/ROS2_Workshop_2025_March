#!/usr/bin/env python

import rospy
from geometry_msgs.pos_msg import Pose

class PositionSubscriber:
    def __init__(self):
        rospy.init_node('position_subscriber', anonymous=True)
        self.pos_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.loginfo("Position Subscriber Node has been started.")

    def pose_callback(self, pos_msg):
        rospy.loginfo(
            f"Turtle Position -> x: {pos_msg.x:.2f}, y: {pos_msg.y:.2f}, theta: {pos_msg.theta:.2f}, "
            f"linear_velocity: {pos_msg.linear_velocity:.2f}, angular_velocity: {pos_msg.angular_velocity:.2f}"
        )

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PositionSubscriber()
        node.spin()
    except rospy.ROSInterruptException:
        pass
