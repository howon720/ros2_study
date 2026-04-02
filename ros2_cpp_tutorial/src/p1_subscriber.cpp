#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")    //  노드 지정
    {
      subscriptionF_ = this->create_subscription<std_msgs::msg::String>(     // this -> ~~ subscription : topic subscriber 생성성
      "fast_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));       //  "topic" : topic 이름

      subscriptionS_ = this->create_subscription<std_msgs::msg::String>(
        "slow_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriptionF_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriptionS_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
