import rclpy
from rclpy.node import Node

from udemy_ros2_pkg.srv import OddEvenCheck


class OddEvenCheckServer(Node):
    def __init__(self):
        super().__init__('odd_even_service_server_node')
        self.srv = self.create_service(
            # Service class, service name, callback function
            OddEvenCheck, "odd_even_check", self.determine_odd_even)

    def determine_odd_even(self, request, response):
        print(f"Request received: {request}")

        # Request and response must match parameters defined in the `srv` file
        if request.number % 2 == 0:
            response.decision = "even"
        elif request.number % 2 == 1:
            response.decision = "odd"
        else:
            response.decision = "error"

        print(f"Response: {response}")

        return response


def main():
    rclpy.init()

    server_node = OddEvenCheckServer()

    print("OddEvenCheck Service Server Running")

    try:
        rclpy.spin(server_node)

    except KeyboardInterrupt:
        # Kill the node
        server_node.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
