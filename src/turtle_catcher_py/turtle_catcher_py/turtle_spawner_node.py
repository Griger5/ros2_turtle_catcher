#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn

import random

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner_node")

        self.spawner_timer = self.create_timer(2.0, self.call_spawn_turtle_service)

    def call_spawn_turtle_service(self):
        client = self.create_client(Spawn, "/spawn")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = Spawn.Request(x = random.uniform(0.1, 10.9), y = random.uniform(0.1, 10.9), theta = random.uniform(-3.14, 3.14))

        future = client.call_async(request)
        future.add_done_callback(self.turtle_spawn_callback)
        
    def turtle_spawn_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = TurtleSpawnerNode()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()