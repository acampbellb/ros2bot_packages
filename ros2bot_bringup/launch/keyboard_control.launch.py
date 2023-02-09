from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # robot keyboard control node
    keyboard_control_node = Node(
        package="ros2bot_controls",
        executable="keyboard_node",
        name="keyboard_control_node",
        parameters=[
            {"linear_speed_limit" : 1.0},
            {"angular_speed_limit" : 5.0}
        ]
    )

    # add nodes to launch description
    ld.add_action(keyboard_control_node)

    # return launch description
    return ld