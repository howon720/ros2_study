from example_interfaces.srv import AddTwoInts  # 여기서 쓸거 가져오는거거

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')  # 노드 이름 지정정
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)  # (type, 'service request 이름', 콜백백 함수 이름)

    def add_two_ints_callback(self, request, response):   # 콜백 함수
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()