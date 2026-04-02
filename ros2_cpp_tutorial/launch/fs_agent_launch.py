from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_cpp_tutorial',
            namespace='',
            executable='fast_agent',
            name='fast_agent'
        ),
        Node(
            package='ros2_cpp_tutorial',
            namespace='',
            executable='slow_agent',
            name='slow_agent'
        ),
        Node(
            package='ros2_cpp_tutorial',
            executable='fast_agent',
            name='fast_agent',
            remappings=[
                ('/fast_agent', '/slow_agent'),
            ]
        )
    ])