#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
import random
from service_server_client.srv import MultipleSpawner, MultipleSpawnerResponse  # Replace with your custom service name

class TurtleSpawnerService:
    def __init__(self):
        rospy.init_node('turtle_spawner_service_server')
        self.service = rospy.Service('spawn_turtles', MultipleSpawner, self.handle_spawn_turtles)
        rospy.loginfo("Turtle Spawner Service is ready.")

    def handle_spawn_turtles(self, req):
        rospy.loginfo(f"Received request to spawn {req.num_robots} turtles with root name '{req.agent_name}'")
        responses = []
        topics_names = rospy.get_published_topics()
        last_number=self.correct_robot_indexes()
        for i in range(req.num_robots):
            turtle_name = f"{req.agent_name}_{last_number+i+1}"
            x, y, theta = self.generate_random_pose()
            rospy.loginfo(f"Spawning turtle: {turtle_name} at ({x}, {y}, {theta})")
            try:
                spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
                spawn_turtle(x, y, theta, turtle_name)
                responses.append(turtle_name)
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to spawn turtle {turtle_name}: {e}")
                responses.append(f"Error: {e}")
        return MultipleSpawnerResponse(responses)

    @staticmethod
    def generate_random_pose():
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)
        theta = random.uniform(0, 2 * 3.14159)
        return x, y, theta
        
    @staticmethod
    def correct_robot_indexes():
        topics_names = rospy.get_published_topics()
        last_number=0
        for topic in topics_names:
            if topic[0].startswith('/turtle'):
                try:
                    number=int(topic[0].split('/')[1].split('_')[1])
                    if number>last_number:
                        last_number=number
                except:
                    pass
        return last_number

if __name__ == '__main__':
    try:
        TurtleSpawnerService()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Turtle Spawner Service shutting down.")