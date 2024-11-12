from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim"
        ),
        Node(
            package="turtle_catcher_py",
            executable="turtle_chaser_node",
            name="turtle_chaser"
        ),
        Node(
            package="turtle_catcher_py",
            executable="turtle_spawner_node",
            name="turtle_spawner"
        )
    ])