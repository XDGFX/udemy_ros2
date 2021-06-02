#!/usr/bin/env python3

# The main ROS Python client library
import rclpy

# The main node class which will be used to create custom nodes
from rclpy.node import Node

# This is for standard message types when defining publishers
from std_msgs.msg import String


class HelloWorldPublisher(Node):
    """
    This is a custom node created by inheriting the Node class from rclpy,
    after initialising it in main().
    """

    def __init__(self):
        # The super() function will run the __init__ class from the passed
        # class Node, and inherit all objects within to this new custom class.
        # It just takes the name of the node as an argument
        super().__init__("hello_world_pub_node")

        # This creates a publisher by using the inherited `create_publisher()` function
        # - We set the message type to string as it will just be publishing strings
        # - The topic will be called "hello_world"
        # - Finally the QoS profile will be set to remember 10 messages in the
        #   event of lost/reconnected connections
        self.pub = self.create_publisher(String, "hello_world", 10)

        # To continually publish at a determined rate, ROS1 use to use a rate
        # object and a `while` loop, but ROS2 added timer functionality to
        # simplify this. It take two parameters:
        # - The rate of the timer in seconds
        # - The callback function
        self.timer = self.create_timer(2, self.publish_hello_world)

        self.counter = 0

    def publish_hello_world(self):
        """
        Publishes data to the "hello_world" topic
        """

        # Initialise the message to send using the correct standard message type
        msg = String()

        # The only attribute is one called `data`, which is the data we want to
        # send with the message
        msg.data = "Hello World! " + str(self.counter)

        # Finally we can publish the message on the topic
        self.pub.publish(msg)

        self.counter += 1


def main():
    # Initialise ROS client library
    # It can take commandline arguments or a context name
    # as input parameters, which we will not use currently.
    rclpy.init()

    my_pub = HelloWorldPublisher()

    print("Publisher Node is now running...")

    try:
        # The `spin` function will keep the function from exiting (I assume
        # because it's all asyncronous now), until a KeyboardInterrupt.
        rclpy.spin(my_pub)

    except KeyboardInterrupt:
        # Kill the node
        my_pub.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
