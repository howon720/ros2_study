#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

// PID 컨트롤러 클래스 정의
class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double compute(double error, double dt) {
        integral_ += error * dt;
        double derivative = (dt > 0) ? (error - prev_error_) / dt : 0.0;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        return output;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

class TurtlebotController : public rclcpp::Node {
public:
    TurtlebotController() : Node("turtlebot_controller"), current_target_idx_(0) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtlebotController::topic_callback, this, _1));
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TurtlebotController::timer_callback, this));
            
        // 움직이는 위치 정하기
        setpoints_ = {{5.5445, 5.5445}, {7.5445, 5.5445}, {7.5445, 7.5445}, {5.5445, 7.5445}};

        linear_pid_ = std::make_shared<PIDController>(0.5, 0.00, 0.1);  // 선속도 PID
        angular_pid_ = std::make_shared<PIDController>(0.7, 0.00, 0.2); // 각속도 PID

        last_time_ = this->now();
    }

private:
    struct State {
        double x = 0;
        double y = 0;
        double theta = 0;
    } turtlebot_state_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    
    std::vector<std::pair<double, double>> setpoints_;
    size_t current_target_idx_;
    
    std::shared_ptr<PIDController> linear_pid_;
    std::shared_ptr<PIDController> angular_pid_;
    
    rclcpp::Time last_time_;

    void topic_callback(const turtlesim::msg::Pose &msg) {
        turtlebot_state_.x = msg.x;
        turtlebot_state_.y = msg.y;
        turtlebot_state_.theta = msg.theta;
    }

    void timer_callback() {
        if (setpoints_.empty()) return;

        // 현재 목표 지점 설정
        auto [target_x, target_y] = setpoints_[current_target_idx_];

        // 목표까지의 거리 및 각도 계산
        double dx = target_x - turtlebot_state_.x;
        double dy = target_y - turtlebot_state_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_theta = std::atan2(dy, dx);
        double angle_error = target_theta - turtlebot_state_.theta;

        // -pi ~ pi 범위로 정규화
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

        // 시간 차이 계산
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // PID 제어 적용
        double linear_vel = (std::abs(angle_error) < 0.1) ? linear_pid_->compute(distance, dt) : 0.0;
        double angular_vel = angular_pid_->compute(angle_error, dt);

        // 속도 명령 생성 및 전송
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear_vel;
        cmd.angular.z = angular_vel;
        publisher_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Moving to target: (%.2f, %.2f) | Distance: %.2f | Angle Error: %.2f",
                    target_x, target_y, distance, angle_error);

        // 목표 지점 도달 시 다음 지점으로 이동
        if (distance < 0.1) {
            current_target_idx_ = (current_target_idx_ + 1) % setpoints_.size();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotController>());
    rclcpp::shutdown();
    return 0;
}