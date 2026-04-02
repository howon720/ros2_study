from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_cpp_tutorial',
            executable='turtlebot_tf2_listener',
            name='listener1',
            parameters=[
                {'target_frame': 'map'},
                {'source_frame': 'turtle1'}
            ]
        )
    ])