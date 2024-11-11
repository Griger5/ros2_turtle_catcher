#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_catcher_interfaces.msg import TurtleArray

import math

class TurtleChaserNode(Node):
    def __init__(self):
        super().__init__("turtle_chaser_node")

        self.curr_x = None
        self.curr_y = None
        self.curr_theta = None
        self.curr_prey = None

        self.curr_position_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_curr_position, 10)

        self.turtles_sub_ = self.create_subscription(TurtleArray, "/turtles", self.callback_turtles, 10)

        self.update_velocity_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.chase_loop = self.create_timer(0.5, self.callback_chase_loop)

    def callback_curr_position(self, msg: Pose):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_theta = msg.theta

    def callback_turtles(self, msg: TurtleArray):
        closest_turtle = None
        curr_distance = None
        
        for turtle in msg.turtles:
            diff_x = self.curr_x - turtle.x
            diff_y = self.curr_y - turtle.y

            distance = math.sqrt(diff_x**2 + diff_y**2)

            if curr_distance == None or distance < curr_distance:
                curr_distance = distance
                closest_turtle = turtle

        self.curr_prey = closest_turtle

    def callback_chase_loop(self):
        if self.curr_prey == None: return
        
        msg = Twist()

        prop_controller_lin = 1
        prop_controller_ang = 3

        diff_x = self.curr_prey.x - self.curr_x
        diff_y = self.curr_prey.y - self.curr_y
        distance = math.sqrt(diff_x**2 + diff_y**2)

        angle = math.atan2(diff_y, diff_x)
        new_angle = angle - self.curr_theta

        if new_angle > math.pi:
            new_angle -= 2 * math.pi
        if new_angle < -math.pi:
            new_angle += 2 * math.pi

        msg.linear.x = prop_controller_lin * distance
        msg.angular.z = prop_controller_ang * new_angle

        self.update_velocity_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = TurtleChaserNode()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()