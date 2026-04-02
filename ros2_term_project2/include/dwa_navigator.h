#ifndef DWA_NAVIGATOR_H
#define DWA_NAVIGATOR_H

#include <geometry_msgs/msg/pose2_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <cmath>
#include <tf2/utils.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct Velocity { double v, w; };
struct Trajectory { Velocity velocity; std::vector<geometry_msgs::msg::Pose2D> path; double score; };

class DwaNavigator {
public:
    explicit DwaNavigator(rclcpp::Node * node);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void setGoal(double gx, double gy);
    void control_loop();

private:
    rclcpp::Node * node_;
    std::vector<float> latest_scan_;
    bool goal_received_{false}; // 처음 시작 할때 움직이는 거 막을라고
    double latest_scan_angle_min_;
    double latest_scan_angle_increment_;
    double latest_scan_max_range_;
    double goal_x_, goal_y_;
    double current_v_{0.0}, current_w_{0.0};  // 경로 늘리기
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    // 경로 시각화 코드 
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr best_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<Velocity> generate_velocity_window(double current_v, double current_w);
    Trajectory simulate_trajectory(double x, double y, double yaw, const Velocity& vel);
    double evaluate_trajectory(const Trajectory& traj, double goal_x, double goal_y);

    // 후보경로 시각화
    void publishCandidateTrajectory(const std::vector<Trajectory>& trajs);
    void publishBestTrajectory(const Trajectory& best_traj);
};

#endif // DWA_NAVIGATOR_H

