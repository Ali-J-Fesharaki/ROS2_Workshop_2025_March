#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from service_server_client.srv import MultipleSpawner

class SpawnServiceClient(Node):
    def __init__(self):
        super().__init__('spawn_service_client')
        self.service_name = 'spawn_turtles'
        self.cli = self.create_client(MultipleSpawner, self.service_name)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service {self.service_name} to become available...')

        self.get_logger().info(f"Connected to service: {self.service_name}")

    def spawn_turtles(self, num_turtles, root_name):
        request = MultipleSpawner.Request()
        request.num_turtles = num_turtles
        request.root_name = root_name

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Response from service: {future.result().turtle_names}")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")

def main(args=None):
    rclpy.init(args=args)
    client = SpawnServiceClient()

    num_turtles = 3  # Number of turtles to spawn
    root_name = "turtle"  # Root name for turtles
    client.spawn_turtles(num_turtles, root_name)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()