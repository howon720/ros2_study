from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_python_tutorial',
            namespace='',
            executable='talker',
            name='talker'
        ),
        Node(
            package='ros2_python_tutorial',
            namespace='',
            executable='listener',
            name='listener'
        )
    ])