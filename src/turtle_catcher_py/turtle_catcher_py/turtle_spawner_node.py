#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtle_catcher_interfaces.msg import Turtle, TurtleArray

import random
from functools import partial

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner_node")

        self.turtles = []

        self.spawner_timer = self.create_timer(2.0, self.call_spawn_turtle_service)

        self.turtles_publisher_ = self.create_publisher(TurtleArray, "/turtles", 10)

    def call_spawn_turtle_service(self):
        client = self.create_client(Spawn, "/spawn")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        turtle_x = random.uniform(0.1, 10.9)
        turtle_y = random.uniform(0.1, 10.9)
        
        request = Spawn.Request(x = turtle_x, y = turtle_y, theta = random.uniform(-3.14, 3.14))

        future = client.call_async(request)
        future.add_done_callback(partial(self.turtle_spawn_callback, x = turtle_x, y = turtle_y))
        
    def turtle_spawn_callback(self, future, x, y):
        try:
            response = future.result()

            new_turtle = Turtle()

            new_turtle.name = response.name
            new_turtle.x = x
            new_turtle.y = y

            self.turtles.append(new_turtle)
            self.publish_turtles()

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def publish_turtles(self):
        msg = TurtleArray()

        msg.turtles = self.turtles

        self.turtles_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TurtleSpawnerNode()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()