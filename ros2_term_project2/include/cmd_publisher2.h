#ifndef CMD_PUBLISHER2_H
#define CMD_PUBLISHER2_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "dwa_navigator.h"

class CmdPublisher : public rclcpp::Node {
public:
    CmdPublisher();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr dwa_timer_; 
    DwaNavigator dwa_nav_;

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif // CMD_PUBLISHER2_H
