from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()
    
    node1 = Node(
        package="my_pkg",
        executable="num_pub_exe"
    )

    node2 = Node(
        package="my_pkg",
        executable="num_counter_exe"
    )

    launch_description.add_action(node1)
    launch_description.add_action(node2)

    return launch_description
