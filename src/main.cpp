// Copyright (c) 2026 Juza. MIT License.
// BNO055 ROS2 Driver — Entry Point

#include "rclcpp/rclcpp.hpp"
#include "bno055_driver/bno055_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bno055_driver::BNO055Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
