#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

struct State {
    double x = 0;
    double y = 0;
    double theta = 0;
};

class TurtlebotController : public rclcpp::Node {
public:
  TurtlebotController() : Node("turtlebot_controller") {
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);  // publisher : input 역할 cmd_vel의 <type>을 가짐짐
  subscription_ = this->create_subscription<turtlesim::msg::Pose>(   // Pose는 이걸로 받음
    "turtle1/pose", 10, std::bind(&TurtlebotController::topic_callback, this, _1));
  timer_ = this->create_wall_timer(
    100ms, std::bind(&TurtlebotController::timer_callback, this));

}

private:
  void timer_callback() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 2.0; // v 속도
    cmd.angular.z = 1.5; // w 각속도도

    RCLCPP_INFO(this->get_logger(), "Turtlebot state: (x: %f, y: %f, theta: %f)", turtlebot_state.x, turtlebot_state.y, turtlebot_state.theta);
    publisher_->publish(cmd);
  }

  void topic_callback(const turtlesim::msg::Pose &msg) {
    turtlebot_state.x = msg.x;
    turtlebot_state.y = msg.y;
    turtlebot_state.theta = msg.theta;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  State turtlebot_state;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotController>());
  rclcpp::shutdown();
  return 0;
}
