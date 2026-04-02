from launch import LaunchDescription  # 관련 모듈 불러오기
from launch_ros.actions import Node

def generate_launch_description():  # launch 실행시 : 안의 노드들이 실행됨
    return LaunchDescription([
        Node(
            package='turtlesim', # 해당 노드 package
            namespace='turtlesim3', # 동일 node 실행시 이름 달라야함함 3
            executable='turtlesim_node', # 실행 프로그램 이름
            name='sim' # 노드이름 바꾸기기??
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim4',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim3/turtle1/pose'),   #앞 위치 보고 뒤에가 따라하기
                ('/output/cmd_vel', '/turtlesim4/turtle1/cmd_vel'),
            ]   # 특정 topic 이름 변경 : ('앞' -> '뒤') 변경
        )
    ])