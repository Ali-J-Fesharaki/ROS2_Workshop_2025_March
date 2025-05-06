#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MultipleSpawner
import time
import sys

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
        request.num_robots = num_turtles
        request.root_name = root_name

        future = self.cli.call_async(request)
        future.add_done_callback(self.future_callback)
        self.timeout_timer = self.create_timer(10.0, self.timeout)
        self.check_timer = self.create_timer(0.1,self.liver)
    def liver(self):
        pass
    def timeout(self):
        pass

    def future_callback(self, future):
        if future.result() is not None and future.done():
            self.get_logger().info(f"Response from service: {future.result().success}")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")
        self.cleanup_timers()

    def cleanup_timers(self):
        self.timeout_timer.cancel()
        self.check_timer.cancel()
        self.destroy_node()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    client = SpawnServiceClient()

    num_turtles = 3  # Number of turtles to spawn
    root_name = "turtle"  # Root name for turtles
    client.spawn_turtles(num_turtles, root_name)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()