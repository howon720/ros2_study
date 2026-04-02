#include "cmd_publi.h"
using std::placeholders::_1;

DwaNavigator::DwaNavigator() : Node("dwa_navigator") {
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&DwaNavigator::scan_callback, this, _1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 목표 goal
    goal_x = 5.0;
    goal_y = 2.0;

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DwaNavigator::control_loop, this));
}

void DwaNavigator::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan = msg->ranges;
    latest_scan_angle_min = msg->angle_min;
    latest_scan_angle_increment = msg->angle_increment;
    latest_scan_max_range = msg->range_max;

    //RCLCPP_INFO(this->get_logger(), "Received scan (%lu points)", msg->ranges.size());  check용 
}

void DwaNavigator::control_loop() {
    geometry_msgs::msg::TransformStamped tf;

    try {
        tf = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero); // this->get_clock()->now(); 오류
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform not available: %s", ex.what());
        return;
    }

    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;

    geometry_msgs::msg::Quaternion q = tf.transform.rotation;
    tf2::Quaternion tf2_q;
    tf2::fromMsg(q, tf2_q);
    double yaw = tf2::getYaw(tf2_q);

    auto candidates = generate_velocity_window(0.0, 0.0);

    Trajectory best_traj;
    best_traj.score = -1e9;

    for (const auto& vel : candidates) {
        Trajectory traj = simulate_trajectory(x, y, yaw, vel);
        traj.score = evaluate_trajectory(traj, goal_x, goal_y);

        if (traj.score > best_traj.score) {
            best_traj = traj;
        }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = best_traj.velocity.v;
    cmd.angular.z = best_traj.velocity.w;
    pub_cmd->publish(cmd);
}


std::vector<Velocity> DwaNavigator::generate_velocity_window(double current_v, double current_w) {
    const double max_v = 1.2;
    const double min_v = 0.0;
    const double max_w = 2.84;
    const double min_w = -2.84;
    const double acc_v = 0.4; // 속도 범위 
    const double acc_w = 2.0;
    const double dt = 0.1;

    std::vector<Velocity> velocities;

    double v_start = std::max(min_v, current_v - acc_v * dt);
    double v_end   = std::min(max_v, current_v + acc_v * dt);
    double w_start = std::max(min_w, current_w - acc_w * dt);
    double w_end   = std::min(max_w, current_w + acc_w * dt);

    for (double v = v_start; v <= v_end; v += 0.01) {
        for (double w = w_start; w <= w_end; w += 0.1) {
            velocities.push_back({v, w});
        }
    }

    return velocities;
}

Trajectory DwaNavigator::simulate_trajectory(double x, double y, double yaw, const Velocity& vel) {
    const double dt = 0.1;
    const double predict_time = 2.0;

    Trajectory traj;
    traj.velocity = vel;

    double px = x;
    double py = y;
    double theta = yaw;

    for (double t = 0.0; t <= predict_time; t += dt) {
        px += vel.v * cos(theta) * dt;
        py += vel.v * sin(theta) * dt;
        theta += vel.w * dt;

        geometry_msgs::msg::Pose2D pose;
        pose.x = px;
        pose.y = py;
        pose.theta = theta;
        traj.path.push_back(pose);
    }

    return traj;
}

double DwaNavigator::evaluate_trajectory(const Trajectory& traj, double goal_x, double goal_y) {
    const auto& last = traj.path.back();
    double dx = goal_x - last.x;
    double dy = goal_y - last.y;
    double goal_cost = std::hypot(dx, dy);  // 거리 짧을수록 좋게

    // 장애물 거리
    double min_obstacle_dist = 1e6;  // 매우 큰 값으로 초기화
    for (const auto& pose : traj.path) {
        for (size_t i = 0; i < latest_scan.size(); ++i) {
            double angle = latest_scan_angle_min + i * latest_scan_angle_increment;
            double laser_dist = latest_scan[i];

            if (laser_dist < 0.05 || laser_dist > latest_scan_max_range) continue;

            // 라이다 빔 방향 좌표 계산
            double rx = laser_dist * cos(angle);
            double ry = laser_dist * sin(angle);

            // 시뮬 경로점과 라이다 점 간 거리
            double dist = std::hypot(pose.x - rx, pose.y - ry);
            min_obstacle_dist = std::min(min_obstacle_dist, dist);
        }
    }

    if (min_obstacle_dist < 0.1) {
        return -1e9;  // 너무 가까운 장애물은 무효
    }

    if (traj.velocity.v == 0.0 && traj.velocity.w == 0.0) {
        return -1e9;  // 정지는 무시
    }

    

    // 속도 유지 점수
    double velocity_score = traj.velocity.v;

    // 총합 점수 
    double score = -goal_cost * 1.0 + min_obstacle_dist * 1.5 + velocity_score * 2.0;  // 가중치 속도 0.5

    RCLCPP_INFO(this->get_logger(), "Score: %.2f (goal: %.2f, obs: %.2f, vel: %.2f)", 
    score, goal_cost, min_obstacle_dist, velocity_score); // check 용

    return score;
}