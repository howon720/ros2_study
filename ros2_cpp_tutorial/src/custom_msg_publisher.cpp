#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_interfaces_tutorial/msg/sphere.hpp"

using namespace std::chrono_literals;

class CustomMsgPublisher : public rclcpp::Node
{
  public:
    CustomMsgPublisher()
    : Node("custom_msg_publisher")
    {
      publisher_ = this->create_publisher<ros2_interfaces_tutorial::msg::Sphere>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&CustomMsgPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = ros2_interfaces_tutorial::msg::Sphere();
      message.center.x = 1;
      message.center.y = 2;
      message.center.z = 3;
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_interfaces_tutorial::msg::Sphere>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomMsgPublisher>());
  rclcpp::shutdown();
  return 0;
}