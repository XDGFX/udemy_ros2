#!/usr/bin/env python3
"""
Part of the project: Publishers & Subscribers
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class WheelSpeedPublisher(Node):
    """
    Publish the wheel speed
    """

    def __init__(self):
        super().__init__("wheel_speed_pub_node")
        self.pub = self.create_publisher(Float32, "/rpm", 10)
        self.timer = self.create_timer(1, self.publish_wheel_speed)

    def publish_wheel_speed(self):
        """
        Publishes wheel speed to the "rpm" topic
        """
        msg = Float32()
        msg.data = float(10)
        self.pub.publish(msg)


def main():
    rclpy.init()

    wheel_speed_publisher = WheelSpeedPublisher()

    print("Publishing wheel speed...")

    try:
        rclpy.spin(wheel_speed_publisher)

    except KeyboardInterrupt:
        # Kill the node
        wheel_speed_publisher.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
