#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

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
    TurtlebotController() : Node("turtlebot_controller"), current_target_idx_(0), reaching_orientation_(false) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtlebotController::topic_callback, this, _1));
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TurtlebotController::timer_callback, this));

        // 위치 & 회전 각각 8개 설정
        setpoints_ = {
            //{5.643, 5.478, 0.0},     // 초기위치
            // {7.544, 5.544, 0.0},     // 오른쪽 이동
            // {7.544, 5.544, M_PI/2},  // 90도 회전
            // {7.445, 7.544, M_PI/2},  // 위쪽 이동
            // {7.445, 7.544, M_PI},    // 180도 회전
            // {5.544, 7.445, M_PI},    // 왼쪽 이동
            // {5.544, 7.445, -M_PI/2},  // -90도 회전
            // {5.643, 5.467, -M_PI/2}  // 아래쪽 이동 (원래 위치로)

            {7.545, 5.545, 0.0},     // 오른쪽 이동
            {7.545, 5.545, M_PI/2},  // 90도 회전
            {7.545, 7.545, M_PI/2},  // 위쪽 이동
            {7.545, 7.545, M_PI},    // 180도 회전
            {5.545, 7.545, M_PI},    // 왼쪽 이동
            {5.545, 7.545, -M_PI/2},                 // -90도 회전
            {5.545, 5.545, -M_PI/2}  // 아래쪽 이동 (원래 위치로)
        };

        linear_pid_ = std::make_shared<PIDController>(0.6, 0.0, 0.5);
        angular_pid_ = std::make_shared<PIDController>(0.6, 0.0, 0.5);
 // 0.6 0.5가 제일 깔끔한듯
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
    
    std::vector<std::tuple<double, double, double>> setpoints_;
    size_t current_target_idx_;
    bool reaching_orientation_;  // 목표 방향을 맞추는 중인지 여부

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

        auto [target_x, target_y, target_theta] = setpoints_[current_target_idx_];

        double dx = target_x - turtlebot_state_.x;
        double dy = target_y - turtlebot_state_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double angle_to_target = std::atan2(dy, dx);
        double angle_error = angle_to_target - turtlebot_state_.theta;
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error)); // -pi ~ pi 정규화

        // 시간 차이 계산
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        geometry_msgs::msg::Twist cmd;

        if (!reaching_orientation_ && (target_x != turtlebot_state_.x || target_y != turtlebot_state_.y)) {
            // 목표 위치로 이동
            double linear_vel = (std::abs(angle_error) < 0.01) ? linear_pid_->compute(distance, dt) : 0.0;
            double angular_vel = angular_pid_->compute(angle_error, dt);

            // 속도 제한
            linear_vel = std::clamp(linear_vel, -1.5, 1.5);
            angular_vel = std::clamp(angular_vel, -2.0, 2.0);

            cmd.linear.x = linear_vel;
            cmd.angular.z = angular_vel;

            RCLCPP_INFO(this->get_logger(), "Moving to: (%.2f, %.2f) | Distance: %.2f | Angle Error: %.2f",
                        target_x, target_y, distance, angle_error);

            // 목표 위치 도달 시 방향 맞추기
            if (distance < 0.01) {
                reaching_orientation_ = true;
            }
        } else {
            // 2️⃣ 목표 방향으로 회전
            double orientation_error = target_theta - turtlebot_state_.theta;
            orientation_error = std::atan2(std::sin(orientation_error), std::cos(orientation_error));

            double angular_vel = angular_pid_->compute(orientation_error, dt);
            angular_vel = std::clamp(angular_vel, -2.0, 2.0);

            cmd.linear.x = 0.0; // 회전만 수행
            cmd.angular.z = angular_vel;

            RCLCPP_INFO(this->get_logger(), "Aligning to angle: %.2f | Error: %.2f", target_theta, orientation_error);

            // 목표 방향 도달 시 다음 지점으로 이동
            if (std::abs(orientation_error) < 0.01) {
                reaching_orientation_ = false;
                current_target_idx_ = (current_target_idx_ + 1) % setpoints_.size();
            }
        }

        publisher_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotController>());
    rclcpp::shutdown();
    return 0;
}