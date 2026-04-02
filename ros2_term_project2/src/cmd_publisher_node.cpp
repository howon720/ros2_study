#include "cmd_publisher2.h"
#include "dwa_navigator.h"

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdPublisher>());
    rclcpp::shutdown();
    return 0;
}
