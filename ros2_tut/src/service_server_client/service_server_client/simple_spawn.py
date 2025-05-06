import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import time

class RandomTurtleSpawner(Node):
    def __init__(self):
        super().__init__('random_turtle_spawner')
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        self.request=Spawn.Request()
        self.spawn_random_turtle()


    def spawn_random_turtle(self):
        self.request.x = random.uniform(0.5, 10.5)
        self.request.y = random.uniform(0.5, 10.5)
        self.request.theta = random.uniform(0, 6.28)  # Random angle in radians
        self.request.name = ''  # Let turtlesim assign a default name

        response = self.client.call_async(self.request)
        time.sleep(1)
        if response is not None:
            self.get_logger().info(f"Turtle spawned successfully: {response.result().name}")
        else:
            self.get_logger().error("Failed to spawn turtle.")

def main(args=None):
    rclpy.init(args=args)
    node = RandomTurtleSpawner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
