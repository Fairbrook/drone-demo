#include <rclcpp/rclcpp.hpp>

#include "px4_control/px4_control_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_control::Px4ControlNode>());
  rclcpp::shutdown();
  return 0;
}
