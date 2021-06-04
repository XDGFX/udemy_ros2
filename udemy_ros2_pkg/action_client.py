#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from udemy_ros2_pkg.action import Navigate2D

distance_threshold = 0.125


class NavigateActionClient(Node):
    def __init__(self):
        super().__init__("navigate_action_server_node")

        # Create an action client, add the action which has been created under
        # `action/Navigate2D.action`, using the topic `navigate`.
        self._action_client = ActionClient(self, Navigate2D, "navigate")

    def send_goal(self, x, y):
        # Initialise goal message
        goal_msg = Navigate2D.Goal()
        goal_msg.goal_point.x = float(x)
        goal_msg.goal_point.y = float(y)
        goal_msg.goal_point.z = float(0)

        # Check that server is ready to receive message
        self._action_client.wait_for_server()

        # Send goal to server asyncronously, with the feedback callback being
        # updated whenever feedback is received.
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # To add a callback when the action is complete
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # Use the name of the feedback within the action file
        print(f"Feedback received: {feedback.distance_to_point}")

    def goal_response_callback(self, future):
        # Check the result of the goal which was sent
        goal_handle = future.result()

        if goal_handle.accepted == False:
            print("Goal rejected")
            return None

        print("Goal accepted")

        # If the goal was accepted, there will be a result. This creates a
        # callback which is run when the result. I'm not sure why this isn't
        # just done within this same function, which itself is a callback when a
        # goal response is received, so this should be investigated in the
        # future. Maybe this callback is once the goal has been received and is
        # in progress? As opposed to once it is complete?

        # Edit: This is correct, this callback is just for the goal initial
        # acceptance, not once it is complete.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Fetch the result value from the result of the future, and extract the
        # variable given within the action file.
        result = future.result().result
        print(f"Result: {result.elapsed_time} seconds")

        # As we have now received the result, there is nothing more for this
        # node to do, so we can shut it down here.
        rclpy.shutdown()


def main():
    rclpy.init()

    action_client = NavigateActionClient()

    print("Action client running...")

    try:
        x = input("Enter an X coordinate: ")
        y = input("Enter a Y coordinate: ")

        action_client.send_goal(x, y)

        rclpy.spin(action_client)

    except KeyboardInterrupt:
        # This will let any connected clients know that the server is about to
        # exit, instead of just killing the node immediately with no warning.
        action_client._action_client.destroy()

        # Kill the node
        action_client.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
