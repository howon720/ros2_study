from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Empty, 'toggle_led', self.toggle_led_callback)
        self.led_state = False
        self.get_logger().info('LED Toggle Service Ready') # 실행하면 ㄱ

    def toggle_led_callback(self,request,response):
        self.led_state = not self.led_state
        self.get_logger().info('LED is now %s' % ('ON' if self.led_state else 'OFF')) # 토글하는거거
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()