#!/usr/bin/env python

import rospy
from service_client.srv import SpawnTurtles, SpawnTurtlesRequest

class SpawnServiceClient:
    def __init__(self):
        rospy.init_node('spawn_service_client')
        self.service_name = 'spawn_turtles'
        rospy.wait_for_service(self.service_name)
        self.spawn_turtles_proxy = rospy.ServiceProxy(self.service_name, SpawnTurtles)
        rospy.loginfo(f"Connected to service: {self.service_name}")

    def spawn_turtles(self, num_turtles, root_name):
        try:
            request = SpawnTurtlesRequest(num_turtles=num_turtles, root_name=root_name)
            response = self.spawn_turtles_proxy(request)
            rospy.loginfo(f"Response from service: {response.turtle_names}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    client = SpawnServiceClient()
    num_turtles = 3  # Number of turtles to spawn
    root_name = "turtle"  # Root name for turtles
    client.spawn_turtles(num_turtles, root_name)