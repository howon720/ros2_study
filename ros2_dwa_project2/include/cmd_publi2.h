#ifndef ROS2_TERM_PROJECT_CMD_PUBLISHER_H
#define ROS2_TERM_PROJECT_CMD_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "map.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <queue>  // For A* algorithm
#include <cmath>  // For std::sqrt, std::abs
#include <tuple>  // For std::tuple

// 경로 탐색 알고리즘 타입

enum RobotState { TURNING, MOVING };

class CmdPublisher : public rclcpp::Node {
public:
  CmdPublisher();

private:
  std::mutex path_mutex;
  void timer_tf_callback();
  void timer_cmd_callback();
  void octomap_callback(const OctomapMsg& octomap_msg);
  int loop_count = 0;               // 루프 카운트
  bool is_computing_path = false;   // 경로 계산 중 상태 플래그
  double x,y,z;
  bool position_updated = false;
  bool algorithm_type=false;
  bool mode = false;
  Map map;
  std::vector<octomap::point3d> path;
  rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
  rclcpp::Subscription<OctomapMsg>::SharedPtr sub_octomap;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  geometry_msgs::msg::Twist cmd_vel;
  double roll, pitch, yaw;
  /////////////////////////////////////// rviz 목표 데이터 수신을 위한 선언 //////////////////////////////////////////
  float goal_x;
  float goal_y;
  bool goal_received;
  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // 목표 수신 콜백 함수
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  /////////////////////////////////////// rviz 목표 데이터 수신을 위한 선언 //////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 경로 생성 함수
  std::vector<octomap::point3d> generate_path_a_star(const octomap::point3d& start, const octomap::point3d& goal);
  std::vector<octomath::Vector3> generate_astar_path(float start_x, float start_y, float goal_x, float goal_y);
  // 경로 이동 함수
  void follow_path(const std::vector<octomap::point3d>& path);

  void visualize_path(const std::vector<octomap::point3d>& path);
  // 경로 시각화를 위한 퍼블리셔
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub;

  bool handle_obstacle(float stop_distance, float reverse_distance);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //wave front
  bool generate_path(const octomap::point3d& start, const octomap::point3d& goal);
  void reconstruct_path(const std::unordered_map<std::string, octomap::point3d>& came_from,
                                    const octomap::point3d& start,
                                    const octomap::point3d& goal) ;
  std::vector<octomap::point3d> get_neighbors(const octomap::point3d& point);
  std::string to_key(const octomap::point3d& point);

  // 깃털 효과 방지 함수 선언
  size_t find_closest_index(const std::vector<octomap::point3d>& path, float x, float y);
    
};
#endif //ROS2_TERM_PROJECT_CMD_PUBLISHER_H