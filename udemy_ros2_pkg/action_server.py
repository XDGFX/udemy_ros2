#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String

from udemy_ros2_pkg.action import Navigate2D

distance_threshold = 0.125


class NavigateActionServer(Node):
    def __init__(self):
        super().__init__("navigate_action_server_node")

        # Create subscriber to get the latest robot position. The callback is to
        # the `update_robot_position` function, and the queue size is 1, as we
        # only care about the latest robot position.
        self.sub = self.create_subscription(
            Point, "robot_potition", self.update_robot_position, 1)

        # Create an action server, add the action which has been created under
        # `action/Navigate2D.action`, using the topic `navigate`, and the
        # callback is `navigate_callback`
        self._action_server = ActionServer(
            self, Navigate2D, "navigate", self.navigate_callback)

        self.robot_current_position =

    def update_robot_position(self, point):
        self.robot_current_position = [point.x, point.y, point.z]

    def navigate_callback(self, goal_handle):
        print("Received goal")

        # This gets the current time, and converts the format to message compatible.
        start_time = self.get_clock().now().to_msg().sec

        robot_goal_point = [
            goal_handle.request.goal_point.x,
            goal_handle.request.goal_point.y,
            goal_handle.request.goal_point.z
        ]

        print(f"Goal point: {robot_goal_point}")

        # Ensure we do not calculate the distance until the current robot
        # position has been updated at least once.
        while self.robot_current_position == None:
            print("Robot current position not yet detected...")

            # This is used instead of time.sleep to prevent any other code being
            # blocked; i.e. it makes everythin asyncronous. If the rest of the
            # code is blocked, the subscriber for current robot position is
            # blocked, and would not be able to update the robot position,
            # meaning we would be stuck in this loop forever!

            # However, adding this spin function within the `main()` function
            # rclpy.spin() can cause issues (recurrent spinning). It has been
            # replaced with the code below.
            rclpy.spin_once(self, timeout_sec=3)

        distance_to_goal = math.inf

        # Initialise the feedback for the action server
        feedback_msg = Navigate2D.Feedback()

        while distance_to_goal > distance_threshold:
            distance_to_goal = math.dist(
                self.robot_current_position, robot_goal_point)

            # The feedback message must match the value specified within the
            # action file
            feedback_msg.distance_to_point = distance_to_goal

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Rate limit
            rclpy.spin_once(self, timeout_sec=1)

        # Goal has been reached
        goal_handle.success()

        # Create result message
        result = Navigate2D.Result()
        result.elapsed_time = float(
            self.get_clock().now().to_msg().sec - start_time)

        return result


def main():
    rclpy.init()

    action_server = NavigateActionServer()

    print("Action server running...")

    try:
        # Can no longer use `rclpy.spin(action_server)` (see comment in `navigate_callback`).
        # Instad, we can use this to loop as long as the node has not been killed.
        while rclpy.ok():
            rclpy.spin_once(action_server)

    except KeyboardInterrupt:
        # This will let any connected clients know that the server is about to
        # exit, instead of just killing the node immediately with no warning.
        action_server._action_server.destroy()

        # Kill the node
        action_server.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
