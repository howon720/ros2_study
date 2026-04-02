import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):  # service server가 있나 존재 확인
        super().__init__('minimal_client_async')  # 노드이름 지정
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')  #클라이언트 생성(service type, 'service 이름')
        while not self.cli.wait_for_service(timeout_sec=1.0):  # 존재 유무 확인
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()  # 초기화화

    def send_request(self, a, b):  # request 보내는 함수
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()  # 존재 확인
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))  #reuest 보내는거
    rclpy.spin_until_future_complete(minimal_client, future)  # 대기대기
    response = future.result()   #들어온 response 저장장
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()