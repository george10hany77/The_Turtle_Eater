from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pub1 = Node(
        package="my_pkg",
        executable="pub_exe",
        name="rbs_proff",
        remappings=[
            ("/my_topic", "/robot_news")
        ],
        parameters=[
            {"message": "Hi from proff"}
        ]
    )

    pub2 = Node(
        package="my_pkg",
        executable="pub_exe",
        name="rbs_denver",
        remappings=[
            ("/my_topic", "/robot_news")
        ],
        parameters=[
            {"message": "Hi from denver"}
        ]
    )

    pub3 = Node(
        package="my_pkg",
        executable="pub_exe",
        name="rbs_tokyo",
        remappings=[
            ("/my_topic", "/robot_news")
        ],
        parameters=[
            {"message": "Hi from tokyo"}
        ]
    )

    pub4 = Node(
        package="my_pkg",
        executable="pub_exe",
        name="rbs_rio",
        remappings=[
            ("/my_topic", "/robot_news")
        ],
        parameters=[
            {"message": "Hi from rio"}
        ]
    )

    pub5 = Node(
        package="my_pkg",
        executable="pub_exe",
        name="rbs_berlin",
        remappings=[
            ("/my_topic", "/robot_news")
        ],
        parameters=[
            {"message": "Hi from berlin"}
        ]
    )

    sub = Node(
        package="my_pkg",
        executable="sub_exe",
        name="smart_phone_station",
        remappings=[
            ("/my_topic", "/robot_news")
        ]
    )

    ld.add_action(pub1)
    ld.add_action(pub2)
    ld.add_action(pub3)
    ld.add_action(pub4)
    ld.add_action(pub5)
    ld.add_action(sub)

    return ld