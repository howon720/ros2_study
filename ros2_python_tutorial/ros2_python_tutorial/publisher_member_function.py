import rclpy
from rclpy.node import Node

from std_msgs.msg import String    # string 인터페이스 사용해서 string 불러옴옴


class MinimalPublisher(Node):      # 클래스 이름름

    def __init__(self):
        super().__init__('minimal_publisher')            # 노드 이름 지정정
        self.publisher_ = self.create_publisher(String, 'topic', 10)      # self.create_publisehers(topic 종류, 'topic 이름', 버퍼 10개)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)     # 타이머 생성     (주기, 호출 함수)
        self.i = 0

    def timer_callback(self):
        msg = String()                                     # publish 타입 : string 선언언
        msg.data = 'Hello World: %d' % self.i              # message 내용
        self.publisher_.publish(msg)                       # message 보내기기
        self.get_logger().info('Publishing: "%s"' % msg.data)    # get 저게 터미널창 print 함수수
        self.i += 1


def main(args=None):
    rclpy.init(args=args)                 # 라이브러리 초기화화

    minimal_publisher = MinimalPublisher()      # 클래스 생성성

    rclpy.spin(minimal_publisher)         # node spin -> 콜백함수 활성화화

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
