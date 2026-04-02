import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, TransformException
import math

class TfListener(Node):
    def __init__(self):
        super().__init__('turtlebot_tf2_listener')

        # Declare and acquire 'target_frame' and 'source_frame' parameters
        self.target_frame = self.declare_parameter('target_frame', 'turtle').value # tf, sf 지정 가능하게 paprameter 선언
        self.source_frame = self.declare_parameter('source_frame', 'world').value

        # Initialize the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # tf 들은거 버퍼에 저장

        # Create a timer to call the callback function
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Try to get the transform between target_frame and source_frame
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )

            # Extract translation and rotation
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation

            # Calculate yaw from quaternion
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            # Log the transformation information
            self.get_logger().info(f'Tf received! (x: {x}, y: {y}, theta: {theta})')

        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = TfListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
