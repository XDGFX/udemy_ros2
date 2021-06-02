#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__("hello_world_sub_node")

        # The `create_subscription` function takes four parameters:
        # - The message type
        # - The topic name
        # - A callback function
        # - QoS profile (subscriber's history depth)
        self.sub = self.create_subscription(
            String, "hello_world", self.subscriber_callback, 10)

    def subscriber_callback(self, msg):
        """
        Callback function for messages received over "hello_world" topic
        """
        print(f"Received: {msg.data}")


def main():
    # Initialise ROS client library
    # It can take commandline arguments or a context name
    # as input parameters, which we will not use currently.
    rclpy.init()

    my_sub = HelloWorldSubscriber()

    print("Waiting for data to be published over topic")

    try:
        # The `spin` function will keep the function from exiting (I assume
        # because it's all asyncronous now), until a KeyboardInterrupt.
        rclpy.spin(my_sub)

    except KeyboardInterrupt:
        # Kill the node
        my_sub.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
