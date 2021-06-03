from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


# Each launch file must have a launch description, which is returned from the
# function `generate_launch_description`.
# It can return multiple nodes. Node that this `Node` object is different from
# the ones we used to create nodes under the package scripts folder.
def generate_launch_description():
    return LaunchDescription([

        # Used to launch a node
        Node(
            package="udemy_ros2_pkg",
            executable="measure_wheel_speed",
            name="measure_wheel_speed_node"
        ),

        # Used to execute an external command
        ExecuteProcess(
            cmd=["ros2", "topic", "list"],
            output="screen"  # Print to terminal
        )
    ])
