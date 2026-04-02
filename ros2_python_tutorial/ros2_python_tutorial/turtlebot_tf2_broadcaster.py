import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class TfBroadcaster(Node):

    def __init__(self):
        super().__init__('turtlebot_tf2_broadcaster')

        # Declare and acquire `turtlename` parameter
        self.turtlename = self.declare_parameter('turtlename', 'turtle').value # 파라미터 선언

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        topic_name = f"{self.turtlename}/pose"  # topic 받기
        self.subscription = self.create_subscription( # subscriber 생성
            Pose,
            topic_name,
            self.handle_turtle_pose,  # 콜백함수 실행
            10
        )

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg() # tf 시간 = 현재시간
        t.header.frame_id = 'world'                      # source frame 지정
        t.child_frame_id = self.turtlename               # target frame 지정

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this is why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)    # tf를 broadcast

def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
