#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;      // timer 시간 사용할 수 있게 해주는거 ms 이런거

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node    // rclcpp 노드 클래스 상속해야 -> 노드 사용 가능
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count1_(0), count2_(0)     // 노드 이름 지정 ""
    {
      publisherF_ = this->create_publisher<std_msgs::msg::String>("fast_topic", 10);   // publisher 정의 : <topic type>("toptic이름", 버퍼 10개 저장해라)
      timerF_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_F_callback, this));     // 타이머 생성 :   ms -> 호출 주기 ,  &콜백함수 이름름

      publisherS_ = this->create_publisher<std_msgs::msg::String>("slow_topic", 10);
      timerS_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_S_callback, this));
    } //500ms : 0.5초초


  private:
    void timer_F_callback()
    {
      auto message = std_msgs::msg::String();        // publish 메시지 string type 설정
      message.data = "Fast timer " + std::to_string(count1_++);     // 메세지 내용
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());   // RCLCPP_INFO : 터미널창 print   
      publisherF_->publish(message);     // 메세지 publish
    }
    void timer_S_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Slow timer " + std::to_string(count2_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisherS_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timerF_; 
    rclcpp::TimerBase::SharedPtr timerS_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherF_;    //  사용하는 publisher랑 타이머 선언  카운트도
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherS_;
    size_t count1_;
    size_t count2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);        // library 초기화
  rclcpp::spin(std::make_shared<MinimalPublisher>());     //  <??>클래스의 노드 생성  ,  spin : 지정 함수 실행될 수 있게 하는 것
  rclcpp::shutdown();    // 종료
  return 0;
}                   // CMakeList 도 수정 해야줘야 터미널에서 명령어로 사용 가능