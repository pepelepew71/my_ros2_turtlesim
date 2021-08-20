from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="turtlesim", executable="turtlesim_node"),
            Node(package="my_ros2_turtlesim", executable="target_node"),
        ]
    )
