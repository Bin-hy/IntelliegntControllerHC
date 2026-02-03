#include <rclcpp/rclcpp.hpp>

#include "hand_control_service.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HandControlService>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}