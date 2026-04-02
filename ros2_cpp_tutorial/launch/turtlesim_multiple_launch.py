from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Launch from this package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros2_cpp_tutorial'),
                    'launch',
                    'turtlesim_mimic_launch.py'
                ])
            ),
        ),

        # Launch from other package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros2_python_tutorial'),
                    'launch',
                    'turtlesim_mimic_launch.py'
                ])
            ),    
        ),
    ])