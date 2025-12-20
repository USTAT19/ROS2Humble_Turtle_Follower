#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')

        # Create a client for /spawn service
        self.client = self.create_client(Spawn, '/spawn')

        # Wait until /spawn is available
        self.get_logger().info('Waiting for /spawn service...')
        self.client.wait_for_service()
        self.get_logger().info('/spawn service available, sending request')

        # Build request
        request = Spawn.Request()
        request.x = 2.0
        request.y = 2.0
        request.theta = 0.0
        request.name = 'turtle2'   # or leave '' to auto-name

        # Call service asynchronously
        self.future = self.client.call_async(request)

        # Timer to check result and then shut down node
        self.timer = self.create_timer(0.1, self.check_result)

    def check_result(self):
        if self.future.done():
            try:
                response = self.future.result()
                self.get_logger().info(f"Spawned turtle named: {response.name}")
            except Exception as e:
                self.get_logger().error(f'Spawn failed: {e}')
            # Done, shut down node
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
