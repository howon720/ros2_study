#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <memory>

bool led_state = false;

void led([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> request,
         [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> response)
{ 
  led_state =! led_state;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LED is now %s", led_state ? "ON" : "OFF");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("led_toggle_server");

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service =
    node->create_service<std_srvs::srv::Empty>("toggle_led", &led);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LED Toggle Service Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}