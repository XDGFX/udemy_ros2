from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Important note: the name stated here will overwrite the name within
        # the node script; resultantly commands which include the node name will
        # need to change correspondingly.
        Node(
            package="udemy_ros2_pkg",
            executable="measure_wheel_speed",
            name="measure_wheel_speed_node"
        ),

        # The `parameters` here will overwrite any default values set within the
        # program file
        Node(
            package="udemy_ros2_pkg",
            executable="calculate_robot_speed",
            name="calculate_robot_speed_node",
            parameters=[
                {"wheel_radius": 0.2}
            ]
        )
    ])
