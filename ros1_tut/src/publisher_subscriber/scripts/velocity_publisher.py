#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class VelocityPublisher:
    def __init__(self):
        rospy.init_node('velocity_publisher', anonymous=True)
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_msg=Twist()
        self.ctrl_c=False
        self.rate = rospy.Rate(0.5) 
        rospy.loginfo("Velocity Publisher Node has been started.")
        rospy.on_shutdown(self.shutdownhook)

    def publish_velocity(self):
        while not self.ctrl_c:
            connections=self.vel_publisher.get_num_connections()
            if connections>0:
                self.vel_msg.linear.x = 1.0  # Move forward
                self.vel_msg.angular.z = 0.5  # Rotate
                self.vel_publisher.publish(self.vel_msg)
                rospy.loginfo(f"Published velocity: linear.x={self.vel_msg.linear.x}, angular.z={self.vel_msg.angular.z}")
                self.rate.sleep()
                
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True


def main():
    try:
        velocity_publisher = VelocityPublisher()
        velocity_publisher.publish_velocity()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node stopped by user.")

if __name__ == '__main__':
    main()
