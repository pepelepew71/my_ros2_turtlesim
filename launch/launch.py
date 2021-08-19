from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    node1 = Node(package="turtlesim", namespace="node1", executable="turtlesim_node")
    ld.add_action(action=node1)
    return ld
