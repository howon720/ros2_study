from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_python_tutorial',
            executable='turtlebot_tf2_listener',
            name='listener1',
            parameters=[
                {'target_frame': 'world'},
                {'source_frame': 'turtle1'}
            ]
        )
    ])