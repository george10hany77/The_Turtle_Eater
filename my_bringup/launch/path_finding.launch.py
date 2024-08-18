from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()
    
    node1 = Node(
        package="my_pkg",
        executable="pf_exe"
    )

    node2 = Node(
        package="my_pkg",
        executable="gtg_exe"
    )

    launch_description.add_action(node2)
    launch_description.add_action(node1)

    return launch_description
