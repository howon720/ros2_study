import rclpy
from rclpy.node import Node

from std_msgs.msg import String    


class MinimalPublisher(Node):      

    def __init__(self):
        super().__init__('minimal_publisher')           
        self.publisherF_ = self.create_publisher(String, 'fast_topic', 10)     
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer1_callback)    
        self.i = 0

        self.publisherS_ = self.create_publisher(String, 'slow_topic', 10)      
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer2_callback)     
        self.i = 0

    def timer1_callback(self):
        msg = String()                                    
        msg.data = 'Fast timer: %d' % self.i             
        self.publisherF_.publish(msg)                     
        self.get_logger().info('Publishing: "%s"' % msg.data)   
        self.i += 1

    def timer2_callback(self):
        msg = String()                                    
        msg.data = 'Slow timer: %d' % self.i              
        self.publisherS_.publish(msg)                     
        self.get_logger().info('Publishing: "%s"' % msg.data)  
        self.i += 1


def main(args=None):
    rclpy.init(args=args)              

    minimal_publisher = MinimalPublisher()    

    rclpy.spin(minimal_publisher)       

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
