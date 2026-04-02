import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_python_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],   # 위에 os.~~ 이거 : launch 폴더 내 모든 파일 install에 설치
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jungwon Park',
    maintainer_email='jungwonpark@seoultech.ac.kr',
    description='ROS2 python tutorial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [               # 여기 밑에 있는거는 package.xml 적힌 내용 같아야함 
            'python_node = ros2_python_tutorial.python_node:main',         
            'talker = ros2_python_tutorial.publisher_member_function:main',    # '실행 프로그램 이름 = package 이름.python 파일 이름'
            'listener = ros2_python_tutorial.subscriber_member_function:main',   # 이친구도
            'double_talker = ros2_python_tutorial.p1_publisher:main',
            'double_listener = ros2_python_tutorial.p1_subscriber:main',
            'fast_agent = ros2_python_tutorial.p2_fast_agent:main',
            'slow_agent = ros2_python_tutorial.p2_slow_agent:main',
            'server = ros2_python_tutorial.service_member_function:main',
            'client = ros2_python_tutorial.client_member_function:main',
            'toggle_server = ros2_python_tutorial.LedToggle_service:main',
            'toggle_client = ros2_python_tutorial.LedToggle_client:main',
            'custom_msg_publisher = ros2_python_tutorial.custom_msg_publisher:main',
            'minimal_param_node = ros2_python_tutorial.python_parameters_node:main',
            'turtlebot_visualizer = ros2_python_tutorial.turtlebot_visualizer:main',
            'turtlebot_tf2_broadcaster = ros2_python_tutorial.turtlebot_tf2_broadcaster:main',
            'turtlebot_tf2_listener = ros2_python_tutorial.turtlebot_tf2_listener:main', 
        ],
    },
)
