import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

from service_server_client.srv import MultipleSpawner, MultipleSpawnerResponse  # Replace with your custom service name

class TurtleSpawnerService(Node):
    def __init__(self):
        super().__init__('turtle_spawner_service_server')
        self.service = self.create_service(MultipleSpawner, 'spawn_turtles', self.handle_spawn_turtles)
        self.get_logger().info("Turtle Spawner Service is ready.")

    def handle_spawn_turtles(self, request, response):
        self.get_logger().info(f"Received request to spawn {request.num_robots} turtles with root name '{request.root_name}'")
        responses = []
        for i in range(request.num_robots):
            turtle_name = f"{request.root_name}_{i+1}"
            x, y, theta = self.generate_random_pose()
            self.get_logger().info(f"Spawning turtle: {turtle_name} at ({x}, {y}, {theta})")
            try:
                spawn_turtle_client = self.create_client(Spawn, '/spawn')
                if not spawn_turtle_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error(f"Service '/spawn' not available.")
                    responses.append(f"Error: Service '/spawn' not available.")
                    continue

                spawn_request = Spawn.Request()
                spawn_request.x = x
                spawn_request.y = y
                spawn_request.theta = theta
                spawn_request.name = turtle_name

                future = spawn_turtle_client.call_async(spawn_request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    responses.append(turtle_name)
                else:
                    self.get_logger().error(f"Failed to spawn turtle {turtle_name}: {future.exception()}")
                    responses.append(f"Error: {future.exception()}")
            except Exception as e:
                self.get_logger().error(f"Failed to spawn turtle {turtle_name}: {e}")
                responses.append(f"Error: {e}")
        response.names = responses
        return response

    @staticmethod
    def generate_random_pose():
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)
        theta = random.uniform(0, 2 * 3.14159)
        return x, y, theta

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Turtle Spawner Service shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()