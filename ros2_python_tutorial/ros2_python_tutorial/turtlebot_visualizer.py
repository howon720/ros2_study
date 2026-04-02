import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from turtlesim.msg import Pose
import math

# 현재 코드에서는 초기값으로 self.previous_point = None을 설정했어.   : 초반 삐죽 튀어나옴

class State:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

class TurtlebotVisualizer(Node):
    def __init__(self):
        super().__init__('turtlebot_visualizer')
        self.publisher_ = self.create_publisher(MarkerArray, 'turtle1/marker', 10)
        self.subscription_ = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.topic_callback,
            10
        )
        self.timer_ = self.create_timer(0.05, self.timer_callback)
        self.turtlebot_state = State()
        self.path_points = [] # 거쳐간 위치 저장 LS

    def timer_callback(self):
        marker_array = MarkerArray()

        # Set marker parameters
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'marker_namespace' 
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.turtlebot_state.x
        marker.pose.position.y = self.turtlebot_state.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(self.turtlebot_state.theta * 0.5)
        marker.pose.orientation.w = math.cos(self.turtlebot_state.theta * 0.5)
        marker.scale.x = 0.7  # 마커 크기
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_array.markers.append(marker)

        # LS
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = 'map'
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.ns = 'line_strip'
        line_strip_marker.id = 1
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.05  # 선의 굵기
        line_strip_marker.color.a = 1.0
        line_strip_marker.color.r = 1.0  # 빨간색 선

        new_point = self.create_point(self.turtlebot_state.x, self.turtlebot_state.y)
        self.path_points.append(new_point)  # 지나간 위치 저장
        line_strip_marker.points = self.path_points  # 모든 경로 적용

        marker_array.markers.append(line_strip_marker)


        self.publisher_.publish(marker_array)

    def topic_callback(self, msg):
        self.turtlebot_state.x = msg.x
        self.turtlebot_state.y = msg.y
        self.turtlebot_state.theta = msg.theta


    def create_point(self, x, y):
         # 좌표를 geometry_msgs/msg/Point 형태로 변환 : LS
        from geometry_msgs.msg import Point
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        return point
    
    

def main(args=None):
    rclpy.init(args=args)
    turtlebot_visualizer = TurtlebotVisualizer()
    rclpy.spin(turtlebot_visualizer)
    turtlebot_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
