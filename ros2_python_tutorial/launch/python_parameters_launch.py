from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_python_tutorial",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen", #프로그램 출력 결과가 터미널 창에 나오도록 설정 
            emulate_tty=True, #프로그램 출력 결과가 터미널 창에 나오도록 설정 
            parameters=[
                {"my_parameter": "earth"} #my_parameter를 earth로 설정
            ]
        )
    ])    