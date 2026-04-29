// Copyright (c) 2026 Juza. MIT License.
// BNO055 ROS2 Node Header
// Lifecycle-aware node that polls the BNO055 and publishes standard ROS2 messages.

#ifndef BNO055_DRIVER__BNO055_NODE_HPP_
#define BNO055_DRIVER__BNO055_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "bno055_driver/bno055_i2c.hpp"

namespace bno055_driver
{

class BNO055Node : public rclcpp::Node
{
public:
  explicit BNO055Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BNO055Node() override = default;

private:
  // ─── Setup ─────────────────────────────────────────────────────────────
  void declareParameters();
  void initSensor();
  void createPublishers();
  void createTimers();

  // ─── Timer callbacks ──────────────────────────────────────────────────
  void imuTimerCallback();
  void diagnosticsTimerCallback();

  // ─── Mode string ↔ enum helpers ────────────────────────────────────────
  static OperationMode operationModeFromString(const std::string & mode_str);
  static std::string operationModeToString(OperationMode mode);

  // ─── Publishers ────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // ─── Timers ────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // ─── Hardware driver ───────────────────────────────────────────────────
  std::unique_ptr<BNO055I2C> imu_;

  // ─── Parameters ────────────────────────────────────────────────────────
  std::string i2c_bus_;
  int i2c_address_;
  std::string frame_id_;
  double update_rate_;
  std::string operation_mode_str_;
  OperationMode operation_mode_;  // Parsed enum for mode-aware logic
  bool publish_diagnostics_;
  bool publish_temperature_;
  bool publish_magnetometer_;
  std::string placement_axis_remap_str_;

  // ─── Covariance values (diagonal) ──────────────────────────────────────
  // Based on BNO055 datasheet noise specifications:
  //   Orientation accuracy: ±1° heading, ±0.5° roll/pitch
  //   Gyroscope noise: ±0.01 rad/s
  //   Accelerometer noise: ±0.06 m/s²
  //   Magnetometer noise: ±0.3 µT
  static constexpr double ORIENTATION_COVARIANCE    = 0.0025;   // ~(0.05 rad)²
  static constexpr double ANGULAR_VEL_COVARIANCE    = 0.0001;   // ~(0.01 rad/s)²
  static constexpr double LINEAR_ACCEL_COVARIANCE   = 0.0036;   // ~(0.06 m/s²)²
  static constexpr double MAGNETIC_FIELD_COVARIANCE  = 0.09;     // ~(0.3 µT)²
};

}  // namespace bno055_driver

#endif  // BNO055_DRIVER__BNO055_NODE_HPP_
