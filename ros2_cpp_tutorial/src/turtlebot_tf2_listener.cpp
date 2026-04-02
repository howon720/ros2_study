#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class TfListener : public rclcpp::Node {
public:
  TfListener() : Node("turtlebot_tf2_listener") {
    // Declare and acquire 'target_frame' and 'source_frame' parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle");  //target frame -> 파라미터 선언
    source_frame_ = this->declare_parameter<std::string>("source_frame", "map");  //source frame -> 파라미터 선언

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());   // listener 생성 부분
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  // 버퍼에 들은거(tf) 10초까지 저장

    timer_ = this->create_wall_timer(
        100ms, std::bind(&TfListener::timer_callback, this));  // 10초
  }

private:
  void timer_callback() {
    geometry_msgs::msg::TransformStamped t; // 위에 저거 frame 있으면 여기다가(TransformStamped t) 저장하는 함수임 

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
      t = tf_buffer_->lookupTransform(
          target_frame_, source_frame_,
          tf2::TimePointZero);  // 제일 최신 tf 가져와라  < 저거 zero가 그거임
    }
    catch (const tf2::TransformException &ex) {   // frame 없으면 이거 실행
      RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          target_frame_.c_str(), source_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::Quaternion q = t.transform.rotation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double theta = std::atan2(siny_cosp, cosy_cosp);

    RCLCPP_INFO(this->get_logger(), "Tf received! (x: %f, y: %f, theta: %f)", 
      t.transform.translation.x, 
      t.transform.translation.y, 
      theta);
}

std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
rclcpp::TimerBase::SharedPtr timer_;
std::string target_frame_, source_frame_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfListener>());
  rclcpp::shutdown();
  return 0;
}