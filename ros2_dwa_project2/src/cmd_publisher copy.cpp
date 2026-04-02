#include "cmd_publi2.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::chrono::steady_clock::time_point last_a_star_time;

CmdPublisher::CmdPublisher() : Node("cmd_publisher"), goal_x(0.0), goal_y(0.0), goal_received(false)  {
  // Publisher
  pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Path visualization
  path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

  // Subscriber
  sub_octomap = this->create_subscription<OctomapMsg>(
                    "octomap_full", 10, std::bind(&CmdPublisher::octomap_callback, this, _1));

  // TF listener
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Timer
  timer_tf = this->create_wall_timer(
            50ms, std::bind(&CmdPublisher::timer_tf_callback, this));

  // 명령 쏴줘 100ms
  timer_cmd = this->create_wall_timer(
      100ms, std::bind(&CmdPublisher::timer_cmd_callback, this));

  // 2D Goal Pose 토픽 구독
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "move_base_simple/goal", 10,
      std::bind(&CmdPublisher::goal_callback, this, std::placeholders::_1));
}

/////////////////////////////////////////// 콜백함수 ///////////////////////////////////////////
void CmdPublisher::timer_tf_callback() {
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer->lookupTransform("map", "base_scan", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform map to base_scan : %s", ex.what());
    return;
  }
  x = t.transform.translation.x;
  y = t.transform.translation.y;
  z = t.transform.translation.z;

  tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  
  m.getRPY(roll, pitch, yaw);

  position_updated = true;

  try {
    // TF 트랜스폼 조회
    geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform("map", "base_scan", tf2::TimePointZero);

    // 현재 위치 업데이트
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    z = t.transform.translation.z;

    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y,
                      t.transform.rotation.z, t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    position_updated = true;

  } catch (const tf2::TransformException &ex) {
      // 예외 발생 시 경고 로그 출력
      RCLCPP_WARN(this->get_logger(), "TF Exception: %s", ex.what());
      position_updated = false;  // 플래그 초기화
  } catch (const std::exception &ex) {
      // 기타 예외 처리
      RCLCPP_ERROR(this->get_logger(), "Unexpected Exception in timer_tf_callback: %s", ex.what());
  }
}

void CmdPublisher::timer_cmd_callback() {
  
  
  ///////////////////////////////////////////A* 경로 생성만 잘 되는 코드////////////////////////////////
     if (!map.is_updated() || !goal_received || !position_updated) {
        RCLCPP_WARN(this->get_logger(), "Map, goal, or position not updated yet.");
        return;
    }
    

    // A* 계산 주기 제한: 2초에 한 번만 실행
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_a_star_time).count() >= 1) {
        // 비동기 A* 실행
        std::thread([this]() {
        octomap::point3d start;

        // 현재 위치에서 기존 경로 중 가장 가까운 지점 인덱스 찾기
        size_t nearest_idx = find_closest_index(path, x, y);
    
        // A* 시작 위치는 기존 경로의 그 이후 3번째 포인트
        if (path.size() > nearest_idx + 3) {
            start = path[nearest_idx + 3];
        } else {
            start = octomap::point3d(x, y, z);
        }
    
        octomap::point3d goal(goal_x, goal_y, z);
        auto new_path = generate_path_a_star(start, goal);
    
        if (new_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to generate path.");
        } else {
            // 기존 경로에서 가까운 점 이후 두 개만 앞에 붙여줌
            std::vector<octomap::point3d> prefix;
    
            if (path.size() > nearest_idx + 1)
                prefix.push_back(path[nearest_idx + 1]);
    
            if (path.size() > nearest_idx + 2)
                prefix.push_back(path[nearest_idx + 2]);
    
            // 앞 + 새 경로 합쳐서 최종 경로로 설정
            prefix.insert(prefix.end(), new_path.begin(), new_path.end());
            path = std::move(prefix);
    
            visualize_path(path);
        }
    }).detach();

        last_a_star_time = now;
  }
    // 경로 따라 이동
      follow_path(path);
      
}

//근처 경로 찾는 함수
size_t CmdPublisher::find_closest_index(const std::vector<octomap::point3d>& path, float x, float y) {
  float min_dist = std::numeric_limits<float>::max();
  size_t closest_index = 0;

  for (size_t i = 0; i < path.size(); ++i) {
      float dx = path[i].x() - x;
      float dy = path[i].y() - y;
      float dist = dx * dx + dy * dy;
      if (dist < min_dist) {
          min_dist = dist;
          closest_index = i;
      }
  }

  return closest_index;
}

void CmdPublisher::octomap_callback(const OctomapMsg& octomap_msg) {
  octomap::point3d world_min(-10, -10, 0);
  octomap::point3d world_max(10, 10, 2);
  map.update(octomap_msg, world_min, world_max);
}

void CmdPublisher::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_x = msg->pose.position.x;
  goal_y = msg->pose.position.y;
  goal_received = true;
  RCLCPP_INFO(this->get_logger(), "Received goal: x = %f, y = %f", goal_x, goal_y);
}

////////////////////////////////////////// 경로생성  /////////////////////////////////////////

std::vector<octomap::point3d> CmdPublisher::generate_path_a_star(const octomap::point3d& start, const octomap::point3d& goal) {
 struct Node {
    octomap::point3d position;
    float cost, score;
    Node* parent;
    bool operator<(const Node& other) const { return score > other.score; }
  };

  // 우선순위 큐와 방문한 노드 집합
  std::priority_queue<Node> open_list;
  std::unordered_map<std::string, float> cost_map;

  auto hash_position = [](const octomap::point3d& point) -> std::string {
    return std::to_string(point.x()) + "," + std::to_string(point.y());
  };

  std::vector<octomap::point3d> path;

  Node start_node = {start, 0.0, static_cast<float>((goal - start).norm()), nullptr};
  open_list.push(start_node);
  cost_map[hash_position(start)] = 0.0;

  // 2D 방향 벡터
  std::vector<octomap::point3d> directions = {
      octomap::point3d(0.125, 0, 0),  // 오른쪽
      octomap::point3d(-0.125, 0, 0), // 왼쪽
      octomap::point3d(0, 0.125, 0),  // 위쪽
      octomap::point3d(0, -0.125, 0)  // 아래쪽
  };

  while (!open_list.empty()) {
    Node current = open_list.top();
    open_list.pop();

    // 목표 지점 도달
    if ((current.position - goal).norm() <= 0.1) {
      Node* node = &current;
      while (node) {
        path.push_back(node->position);
        node = node->parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const auto& dir : directions) {
      octomap::point3d neighbor_position = current.position + dir;
      std::string neighbor_key = hash_position(neighbor_position);

      float distance;
      octomap::point3d closest_obstacle;
      map.get_distance_and_closest_obstacle(neighbor_position, distance, closest_obstacle);

      if (distance < 0.3) {
        continue;
      }

      float new_cost = current.cost + dir.norm();
      if (cost_map.find(neighbor_key) == cost_map.end() || new_cost < cost_map[neighbor_key]) {
        float heuristic = (goal - neighbor_position).norm();
        //float heuristic = (goal - virtual_start).norm(); // 3번재 노드

        Node neighbor = {neighbor_position, new_cost, new_cost + heuristic, new Node(current)};
        open_list.push(neighbor);
        cost_map[neighbor_key] = new_cost;
      }
    }
  }
  return path;
}

void CmdPublisher::visualize_path(const std::vector<octomap::point3d>& path) {
  
  if (path.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot visualize empty path.");
    return;
    }

// 기존 마커 삭제
  {
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "path";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;

    visualization_msgs::msg::MarkerArray clear_array;
    clear_array.markers.push_back(clear_marker);
    path_pub->publish(clear_array);
  }
  // 2. 약간의 시간 대기 (필요 시)
  rclcpp::sleep_for(std::chrono::milliseconds(10));

  // 3. 새 마커 생성
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  for (const auto& point : path) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "path";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = 0.05;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.r = 1.0;
    marker.color.g = 1.0;  // yellow
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_array.markers.push_back(marker);
  }

  path_pub->publish(marker_array);
}

////////////////////////////////////////// 서브 함수  ///////////////////////////////////////


void CmdPublisher::follow_path(const std::vector<octomap::point3d>& path) { 
    // ==== 정적 변수 선언 ====
    static size_t current_waypoint_index = 1; 
    static bool is_turning = true;             
    static float prev_distance = 0.0f; 
    static float prev_angle_diff = 0.0f;  // 회전 D 제어용

    // ==== 제어 파라미터 ====
    const float distance_threshold = 0.1f; 
    const float angle_threshold    = 0.1f; 

    // PD 게인 (필요 시 조정)
    const float Kp_dist = 3.f;  //3.   
    const float Kd_dist = 0.5f;  //0.5 
    const float Kp_angle = 1.2f;   // 각도가 잘 안 줄면 값 증가  1.2
    const float Kd_angle = 0.25f;  //0.25

    // 속도 제한
    const float max_linear_speed  = 1.0f;   //1.0
    const float max_angular_speed = 1.5f;  // 회전 속도 좀 높여볼 수 있음 1.5

    static std::vector<octomap::point3d> previous_path;

    // 새로운 경로 업데이트
    if (path != previous_path) {
        RCLCPP_INFO(this->get_logger(), "Path updated. Resetting navigation...");
        current_waypoint_index = 1; 
        is_turning = true; 
        previous_path = path;
    }

    // 경로 체크
    if (path.empty() || current_waypoint_index >= path.size() - 1) {
        RCLCPP_INFO(this->get_logger(), "Path traversal done. Stopping.");
        geometry_msgs::msg::Twist stop_cmd;
        pub_cmd->publish(stop_cmd);
        return;
    }

    // ---- 현재 노드와 다음 노드 중점 계산 ----
    const octomap::point3d& curr_wp = path[current_waypoint_index];
    const octomap::point3d& next_wp = path[current_waypoint_index + 1];
    float mid_x = 0.5f * (curr_wp.x() + next_wp.x());
    float mid_y = 0.5f * (curr_wp.y() + next_wp.y());

    // 로봇에서 중점까지의 거리/방향
    float dx = mid_x - x;
    float dy = mid_y - y;
    float distance = std::sqrt(dx * dx + dy * dy);
    float angle_to_target = std::atan2(dy, dx);
    float angle_diff = angle_to_target - yaw;

    // -π ~ π 범위로 정규화
    while (angle_diff > M_PI)  angle_diff -= 2.0f * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;

    // PD용 에러 계산
    float distance_error = distance;
    float distance_derivative = distance - prev_distance;

    float angle_error = angle_diff;
    float angle_derivative = angle_diff - prev_angle_diff;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0f;
    cmd_vel.angular.z = 0.0f;

    // === TURNING 모드 ===
    if (is_turning) 
    {
        RCLCPP_INFO(this->get_logger(), "Mode: TURNING | Waypoint idx: %lu/%lu", 
                    current_waypoint_index, path.size() - 1);
        RCLCPP_INFO(this->get_logger(), " 각도 차이 : %f", angle_diff);
        RCLCPP_INFO(this->get_logger(), " 요 값이 잘 들어오나 확인 : %f", yaw);

        // PD 제어(회전)
        float angular_cmd = Kp_angle * angle_error + Kd_angle * angle_derivative;
        // 속도 제한
        angular_cmd = std::clamp(angular_cmd, -max_angular_speed, max_angular_speed);
        cmd_vel.angular.z = angular_cmd;

        // 각도 오차가 충분히 작으면 FORWARD 모드로 전환
        if (std::fabs(angle_diff) < angle_threshold) {
            RCLCPP_INFO(this->get_logger(), "Switching to FORWARD mode.");
            is_turning = false;
        }
    }
    // === FORWARD 모드 ===
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Mode: FORWARD | Waypoint idx: %lu/%lu", 
                    current_waypoint_index, path.size() - 1);
        RCLCPP_INFO(this->get_logger(), " 거리 : %f, 각도 차이 : %f", distance, angle_diff);

        // 전진 PD 제어
        float linear_cmd = Kp_dist * distance_error + Kd_dist * distance_derivative;
        linear_cmd = std::clamp(linear_cmd, 0.0f, max_linear_speed);
        cmd_vel.linear.x = linear_cmd;

        // 주행 중에도 각도 보정(간단 P 제어)
        float angular_cmd = Kp_angle * angle_error;
        angular_cmd = std::clamp(angular_cmd, -max_angular_speed, max_angular_speed);
        cmd_vel.angular.z = angular_cmd;

        // 중점 도달 시 다음으로
        if (distance < distance_threshold) {
            RCLCPP_INFO(this->get_logger(), "Reached midpoint. Next waypoint...");
            current_waypoint_index++;
            is_turning = true;
        }
    }

    // 퍼블리시
    pub_cmd->publish(cmd_vel);

    // 과거 값 업데이트
    prev_distance = distance;
    prev_angle_diff = angle_diff;
}






  