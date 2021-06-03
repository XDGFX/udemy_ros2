import rclpy
from rclpy.node import Node

from udemy_ros2_pkg.srv import OddEvenCheck


class OddEvenCheckClient(Node):
    def __init__(self):
        super().__init__('odd_even_service_client_node')

        # Match name with server
        self.client = self.create_client(OddEvenCheck, "odd_even_check")

        # Empty request message to be filled with data
        self.req = OddEvenCheck.Request()

    def send_request(self, num):
        # Load number into request message
        self.req.number = int(num)

        # Wait for the service to become available
        self.client.wait_for_service()

        # Send call and assign future
        self.future = self.client.call_async(self.req)

        # Wait for response (synchronous)
        rclpy.spin_until_future_complete(self, self.future)

        # Retrieve response object
        self.result = self.future.result()

        return self.result


def main():
    rclpy.init()

    client_node = OddEvenCheckClient()

    print("OddEvenCheck Service Client Making A Request...")

    try:
        # Prompt for variable
        user_input = input("Enter an integer: ")

        res = client_node.send_request(user_input)
        print(f"Server result: {res.decision}")

    except KeyboardInterrupt:
        # Kill the node
        client_node.destroy_node()

        # Shutdown and disconnect the client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()
