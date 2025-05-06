#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import time

from custom_interfaces.srv import MultipleSpawner    # Replace with your custom service name

class TurtleSpawnerService(Node):
    def __init__(self):
        super().__init__('turtle_spawner_service_server')
        self.service = self.create_service(MultipleSpawner, 'spawn_turtles', self.handle_spawn_turtles)
        self.get_logger().info("Turtle Spawner Service is ready.")
        self.handler_spawn_service=self.create_client(Spawn, '/spawn')
        self.counter_response=0
        self.response=MultipleSpawner.Response()
        self.request=MultipleSpawner.Request()
    def handle_spawn_turtles(self, request, response):
        self.futures=[]
        self.get_logger().info(f"Received request to spawn {request.num_robots} turtles with root name '{request.root_name}'")
        self.counter_response=0
        last_number=self.correct_robot_indexes()
        for i in range(request.num_robots):
            turtle_name = f"{request.root_name}_{last_number+i+1}"
            x, y, theta = self.generate_random_pose()
            self.get_logger().info(f"Spawning turtle: {turtle_name} at ({x}, {y}, {theta})")
            spawn_request = Spawn.Request()
            spawn_request.x = x
            spawn_request.y = y
            spawn_request.theta = theta
            spawn_request.name = turtle_name  
            future = self.handler_spawn_service.call_async(spawn_request)
            future.add_done_callback(self.adding_results_to_list)
        self.timeout_timer = self.create_timer(10.0, self.on_timeout)
        self.check_timer = self.create_timer(0.1, self.check_if_all_done)

        return self.response  # This will complete later


    def check_if_all_done(self):
        if self.counter_response >= self.request.num_robots:
            self.get_logger().info("All turtles spawned successfully.")
            self.cleanup_timers()
            print(self.futures)
            self.response.success = self.futures

    def cleanup_timers(self):
        self.timeout_timer.cancel()
        self.check_timer.cancel()

    def on_timeout(self):
        self.get_logger().warn("Timeout while waiting for turtles to spawn.")
        self.cleanup_timers()
        self.response.success = self.futures


    def adding_results_to_list(self,future):
        temp=future.result().name is not None
        print(temp)
        self.counter_response+=1
        self.futures.append(temp)

    @staticmethod
    def generate_random_pose():
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)
        theta = random.uniform(0, 2 * 3.14159)
        return x, y, theta

    def correct_robot_indexes(self):
        topics_names = self.get_topic_names_and_types()
        last_number = 0
        for topic, _ in topics_names:
            if topic.startswith('/turtle'):
                try:
                    number = int(topic.split('/')[1].split('_')[1])
                    if number > last_number:
                        last_number = number
                except (IndexError, ValueError):
                    pass
        return last_number
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()