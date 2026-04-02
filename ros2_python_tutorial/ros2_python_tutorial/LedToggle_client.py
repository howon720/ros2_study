import sys

from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Empty, 'toggle_led')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Empty.Request()

    def send_request(self):
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request()
    rclpy.spin_until_future_complete(minimal_client, future)
    minimal_client.get_logger().info(
        'Service call completed')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()