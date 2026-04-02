#include "cmd_publisher2.h"
#include "dwa_navigator.h"


// CmdPublisher
CmdPublisher::CmdPublisher()
: Node("cmd_publisher")
, dwa_nav_(this)
{
    // LaserScan 구독
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&DwaNavigator::scan_callback, &dwa_nav_, std::placeholders::_1));

    // Goal 구독
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal", 10,
        std::bind(&CmdPublisher::goal_callback, this, std::placeholders::_1));

    // DWA 제어 루프 타이머
    dwa_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DwaNavigator::control_loop, &dwa_nav_));
}

void CmdPublisher::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    dwa_nav_.setGoal(msg->pose.position.x, msg->pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f, y=%.2f",
                msg->pose.position.x, msg->pose.position.y);
}



// DwaNavigator 구현
DwaNavigator::DwaNavigator(rclcpp::Node* node)
: node_(node), goal_x_(0.0), goal_y_(0.0)
{
    pub_cmd_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    traj_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("dwa_candidates", 1);
    best_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("dwa_best", 1);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void DwaNavigator::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg->ranges;
    latest_scan_angle_min_ = msg->angle_min;
    latest_scan_angle_increment_ = msg->angle_increment;
    latest_scan_max_range_ = msg->range_max;
}

void DwaNavigator::setGoal(double gx, double gy) {
    goal_x_ = gx;
    goal_y_ = gy;
    goal_received_ = true; 
}

void DwaNavigator::control_loop() {
    
  // 목표 안 받았으면 안움직임 : 처음에 그냥 움직이는거 잡기
    if (!goal_received_){
      geometry_msgs::msg::Twist stop{};
      pub_cmd_->publish(stop);
      return;
    }

    geometry_msgs::msg::TransformStamped tfst;
    try {
        tfst = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "TF transform not available: %s", ex.what());
        return;
    }

    // 현재 위치 방향 계산
    double x = tfst.transform.translation.x;
    double y = tfst.transform.translation.y;
    tf2::Quaternion tf2_q;
    tf2::fromMsg(tfst.transform.rotation, tf2_q);
    double yaw = tf2::getYaw(tf2_q);

    // goal 도달 판정 : 새로운 goal 받았을 때 stop 조정
    double dx = goal_x_ - x;
    double dy = goal_y_ - y;
    double dist_to_goal = std::hypot(dx, dy);
    const double GOAL_TOL = 0.1;  // 10cm 이내 : 도달로 간주
    if (dist_to_goal < GOAL_TOL) {
        RCLCPP_INFO(node_->get_logger(), "Goal reached (%.3f m). Stopping.", dist_to_goal);
        // 정지 명령
        pub_cmd_->publish(geometry_msgs::msg::Twist{});
        // 내부 상태 리셋
        goal_received_ = false;
        current_v_ = 0.0;
        current_w_ = 0.0;
        return;
    }

    // Lidar 최소 거리 계산 출력
    if (!latest_scan_.empty()) {
    // 유효 거리
        double min_d = std::numeric_limits<double>::infinity();
        for (double d : latest_scan_) {
            if (d >= 0.05 && d <= latest_scan_max_range_) {
                min_d = std::min(min_d, d);
            }
        }
        if (min_d < std::numeric_limits<double>::infinity()) {
            RCLCPP_INFO(node_->get_logger(),
                  "Closest obstacle distance (current scan): %.2f m", min_d);
        } else {
            RCLCPP_INFO(node_->get_logger(),
                  "No valid laser returns");
        }
    }

    auto candidates = generate_velocity_window(current_v_, current_w_);

    // 경로 생산중인지
    //RCLCPP_INFO(node_->get_logger(), "DWA: %zu candidates", candidates.size());

    std::vector<Trajectory> all_trajs;
    Trajectory best_traj;
    best_traj.score = -1e9;

    for (const auto& vel : candidates) {
        Trajectory traj = simulate_trajectory(x, y, yaw, vel);
        traj.score = evaluate_trajectory(traj, goal_x_, goal_y_);
        all_trajs.push_back(traj);
        if (traj.score > best_traj.score) best_traj = traj;
    }

    // BEST 궤적 정보 check
    RCLCPP_INFO(node_->get_logger(),
    "BEST → score: %.2f, v: %.2f, w: %.2f",
    best_traj.score, best_traj.velocity.v, best_traj.velocity.w);

    // 후보 경로 시각화
    publishCandidateTrajectory(all_trajs);

    // 최적 궤적 시각화
    publishBestTrajectory(best_traj);

    // cmd_vel 퍼블리시
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = best_traj.velocity.v;
    cmd.angular.z = best_traj.velocity.w;
    pub_cmd_->publish(cmd);

    // 현재 속도 각속도 알기
    current_v_ = cmd.linear.x;
    current_w_ = cmd.angular.z;
}

// 후보경로 시각화
void DwaNavigator::publishCandidateTrajectory(const std::vector<Trajectory>& trajs) {
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  for (const auto& traj : trajs) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = node_->now();
    m.ns    = "dwa_candidates";
    m.id    = id++;
    m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action= visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0;
    m.color.a = 0.3;
    // 궤적 점들 추가
    for (const auto& pose : traj.path) {
      geometry_msgs::msg::Point p;
      p.x = pose.x; p.y = pose.y; p.z = 0.0;
      m.points.push_back(p);
    }
    marker_array.markers.push_back(m);
  }
  traj_pub_->publish(marker_array);
}

// 최적 경로 시각화
void DwaNavigator::publishBestTrajectory(const Trajectory& best_traj) {
  visualization_msgs::msg::MarkerArray best_array;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = node_->now();
  m.ns    = "dwa_best";
  m.id    = 0;
  m.type  = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action= visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.05;
  m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0;
  m.color.a = 1.0;

  for (const auto& pose : best_traj.path) {
    geometry_msgs::msg::Point p;
    p.x = pose.x;
    p.y = pose.y;
    p.z = 0.05;
    m.points.push_back(p);
  }
  best_array.markers.push_back(m);
  best_pub_->publish(best_array);
}


std::vector<Velocity> DwaNavigator::generate_velocity_window(double current_v, double current_w) {

    const double max_v = 0.2, min_v = 0.0, max_w = 2.84, min_w = -2.84;   //2.84
                        // 0.3
    // 여기 수정 필요 : 너무 제한 빡빡하면 속도 제한을 못검 -> +값 -값 가속도 제한 다르게 걸어도 될듯
    const double max_acc_v = 2.0, max_acc_w = 3.0, dt = 0.1;  
    // 0.2     //1.0

    double v_start = std::max(min_v, current_v - max_acc_v*dt);
    double v_end   = std::min(max_v, current_v + max_acc_v*dt);
    double w_start = std::max(min_w, current_w - max_acc_w*dt);
    double w_end   = std::min(max_w, current_w + max_acc_w*dt);

    std::vector<Velocity> vel;
    for (double v = v_start; v <= v_end; v += 0.01)
        for (double w = w_start; w <= w_end; w += 0.05)
            vel.push_back({v,w});
    return vel;
}

// 경로 예측
Trajectory DwaNavigator::simulate_trajectory(double x, double y, double yaw, const Velocity& vel) {
    // 예측시간 조정
    const double dt = 0.1, predict_time = 1.5;   // 2.0
    Trajectory traj; traj.velocity = vel;
    double px = x, py = y, theta = yaw;

    for (double t=0; t<=predict_time; t+=dt) {
        px += vel.v * std::cos(theta) * dt;
        py += vel.v * std::sin(theta) * dt;
        theta += vel.w * dt;
        geometry_msgs::msg::Pose2D p; p.x=px; p.y=py; p.theta=theta;
        traj.path.push_back(p);
    }
    return traj;
}

double DwaNavigator::evaluate_trajectory(const Trajectory& traj, double gx, double gy) {
    
    // goal 거리
    const auto &last = traj.path.back();
    double goal_cost = std::hypot(gx-last.x, gy-last.y);
    
    // 장애물 거리
    double min_obs = 1e9; // 일단 무한대 지정

    for (const auto &p : traj.path) {
      for (size_t i = 0; i < latest_scan_.size(); ++i) {
        double angle = latest_scan_angle_min_ + i * latest_scan_angle_increment_;
        double d = latest_scan_[i];
        if (d < 0.05 || d > latest_scan_max_range_) continue;
        double rx = d * std::cos(angle), ry = d * std::sin(angle);
        min_obs = std::min(min_obs, std::hypot(p.x - rx, p.y - ry));
      }
    }

    // 후보궤적 체크 
    // 안전 거리 조정 (5cm)
    constexpr double SAFETY_DIST = 0.20;  
    if (min_obs < 0.10) {  // 진짜 가까운 건 완전 배제
      return -1e9; 
    }

    // 장애물 페널티
    double obs_penalty = 0.0;
    if (min_obs < SAFETY_DIST) {
      obs_penalty = (SAFETY_DIST - min_obs) * 10.0;  
    }

    //if (traj.velocity.v == 0 && traj.velocity.w == 0) return -1e9; // 완전 정지 X  
    
    // score 계산식
    double score = -goal_cost*0.5 + min_obs*0.8;
    // double score = -goal_cost*0.5 + min_obs*0.8 + traj.velocity.v*0.4 - obs_penalty;

    // check용
    // RCLCPP_DEBUG(node_->get_logger(),
    //   "EVAL → goal:%.2f min_obs:%.2f v:%.2f => score:%.2f",
    //                 goal_cost, min_obs, traj.velocity.v, score);

    return score;
}
