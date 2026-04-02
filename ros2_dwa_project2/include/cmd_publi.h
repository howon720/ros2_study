#ifndef CMD_PUBLI_H
#define CMD_PUBLI_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>  
#include <cmath>
#include <limits>
#include <tf2/utils.h>
#include <vector>
#include <memory>

struct Velocity {
    double v;
    double w;
  };
  
struct Trajectory {
    std::vector<geometry_msgs::msg::Pose2D> path;
    double score;
    Velocity velocity;
  };


class DwaNavigator : public rclcpp::Node {
public:
    DwaNavigator();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void control_loop();

    std::vector<Velocity> generate_velocity_window(double current_v, double current_w);
    Trajectory simulate_trajectory(double x, double y, double yaw, const Velocity& vel);
    double evaluate_trajectory(const Trajectory& traj, double goal_x, double goal_y);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::vector<float> latest_scan;
    double latest_scan_angle_min;
    double latest_scan_angle_increment;
    double latest_scan_max_range;
    double goal_x, goal_y;
};

#endif 