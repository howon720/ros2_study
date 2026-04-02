from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='ros2_cpp_tutorial',
            namespace='turtlesim1',
            executable='turtlebot_controller',
            name='controller'
        )
    ])