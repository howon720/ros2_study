import rclpy
from rclpy.node import Node

from ros2_interfaces_tutorial.msg import Sphere

class CustomMsgPublisher(Node):

    def __init__(self):
        super().__init__('custom_msg_publisher')
        self.publisher_ = self.create_publisher(Sphere, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Sphere()
        msg.center.x = 1.0
        msg.center.y = 2.0
        msg.center.z = 3.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CustomMsgPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()