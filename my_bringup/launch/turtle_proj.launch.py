from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtle_node = Node(
        name="The_Killer_Turtle",
        package="turtlesim",
        executable="turtlesim_node"
    )

    controller_node = Node(
        name="Turtle_Controller",
        package="turtle_project",
        executable="t_c_exe"
    )

    spawner_node = Node(
        name="The_Spawner",
         package="turtle_project",
        executable="t_s_exe"
    )

    ld.add_action(turtle_node)
    ld.add_action(controller_node)
    ld.add_action(spawner_node)

    return ld