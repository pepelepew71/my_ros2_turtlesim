from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(package="turtlesim", executable="turtlesim_node")
    node2 = Node(package="my_ros2_turtlesim", executable="tracking")

    ld = LaunchDescription()
    ld.add_action(node1)
    ld.add_action(node2)
    return ld
