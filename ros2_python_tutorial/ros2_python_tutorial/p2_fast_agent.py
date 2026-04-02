import rclpy
from rclpy.node import Node

from std_msgs.msg import String    


class FastAgent(Node):      

    def __init__(self):
        super().__init__('fast_agent')           
        self.publisher_ = self.create_publisher(String, 'fast_topic', 10)     
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)    
        self.i = 0

        self.subscriptionS = self.create_subscription(
            String,
            'slow_topic',
            self.listener_callback,
            10)
        self.subscriptionS

    def timer_callback(self):
        msg = String()                                    
        msg.data = 'Fast timer: %d' % self.i             
        self.publisher_.publish(msg)                     
        self.get_logger().info('Publishing: "%s"' % msg.data)   
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)              

    fast_agent = FastAgent()    

    rclpy.spin(fast_agent)       

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fast_agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
