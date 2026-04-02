cmake_minimum_required(VERSION 3.5)
project(ros2_cpp_tutorial)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)           # 사용하는 그거 있어야함, package.xml이랑 연동 되야함
find_package(ros2_interfaces_tutorial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(talker src/publisher_member_function.cpp)      # 터미널에서 명령어 설치
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

add_executable(double_talker src/p1_publisher.cpp)    
ament_target_dependencies(double_talker rclcpp std_msgs)
install(TARGETS
  double_talker
  DESTINATION lib/${PROJECT_NAME})


add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

add_executable(double_listener src/p1_subscriber.cpp)
ament_target_dependencies(double_listener rclcpp std_msgs)
install(TARGETS
  double_listener
  DESTINATION lib/${PROJECT_NAME})

  add_executable(fast_agent src/p2_fast_agent.cpp)    
ament_target_dependencies(fast_agent rclcpp std_msgs)
install(TARGETS
  fast_agent
  DESTINATION lib/${PROJECT_NAME})

  add_executable(slow_agent src/p2_slow_agent.cpp)    
ament_target_dependencies(slow_agent rclcpp std_msgs)
install(TARGETS
  slow_agent
  DESTINATION lib/${PROJECT_NAME})

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)
install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)
install(TARGETS
  client
  DESTINATION lib/${PROJECT_NAME})

add_executable(server_led src/led_toggle_server.cpp)
ament_target_dependencies(server_led rclcpp std_srvs)
install(TARGETS
  server_led
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(client_led src/led_toggle_client.cpp)
ament_target_dependencies(client_led rclcpp std_srvs)
install(TARGETS
  client_led
  DESTINATION lib/${PROJECT_NAME})


add_executable(custom_msg_publisher src/custom_msg_publisher.cpp)
ament_target_dependencies(custom_msg_publisher
  rclcpp
  ros2_interfaces_tutorial
)
install(TARGETS
  custom_msg_publisher
  DESTINATION lib/${PROJECT_NAME})  


add_executable(turtlebot_controller src/turtlebot_controller.cpp)
ament_target_dependencies(turtlebot_controller rclcpp geometry_msgs turtlesim)
install(TARGETS 
  turtlebot_controller
  DESTINATION lib/${PROJECT_NAME})

add_executable(turtlebot_square_controller src/turtlebot_square_controller.cpp)
ament_target_dependencies(turtlebot_square_controller rclcpp geometry_msgs turtlesim)
install(TARGETS 
  turtlebot_square_controller
  DESTINATION lib/${PROJECT_NAME})

add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)
install(TARGETS
  minimal_param_node
  DESTINATION lib/${PROJECT_NAME}) 

add_executable(turtlebot_visualizer src/turtlebot_visualizer.cpp)
ament_target_dependencies(turtlebot_visualizer rclcpp geometry_msgs turtlesim visualization_msgs)
install(TARGETS 
  turtlebot_visualizer
  DESTINATION lib/${PROJECT_NAME})

add_executable(turtlebot_tf2_broadcaster src/turtlebot_tf2_broadcaster.cpp)
ament_target_dependencies(
  turtlebot_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)  
install(TARGETS
  turtlebot_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

add_executable(turtlebot_tf2_listener src/turtlebot_tf2_listener.cpp)
ament_target_dependencies(
  turtlebot_tf2_listener
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)  
install(TARGETS
  turtlebot_tf2_listener
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch 
        DESTINATION share/${PROJECT_NAME})


ament_package()
