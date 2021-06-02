#!/usr/bin/env python3
"""
Part of the project: Publishers & Subscribers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from math import pi

wheel_radius = 0.125  # In m


class CalcRobotSpeed(Node):
    def __init__(self):
        super().__init__("wheel_speed_sub_node")

        # Setup the subscriber to the "/rpm" topic
        self.sub = self.create_subscription(
            Float32, "/rpm", self.rpm2speed, 10
        )

        # Setup the publisher for the "/speed" topic
        self.pub = self.create_publisher(
            Float32, "/speed", 10
        )

    def rpm2speed(self, msg):
        """
        Callback function for messages received over "rpm" topic
        """

        print(f"Received rpm: {msg.data}")

        # Convert from rpm to ground speed
        input_rpm = msg.data
        output_speed = 2 * pi * wheel_radius * input_rpm / 60  # Speed in m/s

        print(f"Calculated robot speed: {output_speed}")

        # Publish on the "/speed" topic
        msg = Float32()
        msg.data = float(output_speed)
        self.pub.publish(msg)


def main():
    rclpy.init()

    calculate_robot_speed = CalcRobotSpeed()

    print("Waiting for motor rpm to convert...")

    try:
        rclpy.spin(calculate_robot_speed)

    except KeyboardInterrupt:
        # Kill the node
        calculate_robot_speed.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
