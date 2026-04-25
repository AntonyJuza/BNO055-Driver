// Copyright (c) 2026 Juza. MIT License.
// BNO055 ROS2 Node Implementation
//
// Polls the BNO055 over I2C at a configurable rate and publishes:
//   - sensor_msgs/Imu       on /imu/bno055           (orientation + gyro + accel)
//   - sensor_msgs/Temperature on /imu/bno055/temperature
//   - diagnostic_msgs/DiagnosticArray on /imu/bno055/status (calibration)
//
// Designed for integration with robot_localization EKF for SLAM mapping.

#include "bno055_driver/bno055_node.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace bno055_driver
{

BNO055Node::BNO055Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("bno055_node", options)
{
  RCLCPP_INFO(get_logger(), "BNO055 IMU Driver Node starting...");

  declareParameters();
  createPublishers();
  initSensor();
  createTimers();
}

// ─── Parameter Declaration ───────────────────────────────────────────────────

void BNO055Node::declareParameters()
{
  // I2C configuration
  i2c_bus_ = declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
  i2c_address_ = declare_parameter<int>("i2c_address", 0x28);
  
  // ROS2 configuration
  frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
  update_rate_ = declare_parameter<double>("update_rate", 50.0);
  
  // Sensor configuration
  operation_mode_str_ = declare_parameter<std::string>("operation_mode", "NDOF");
  
  // Feature toggles
  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);
  publish_temperature_ = declare_parameter<bool>("publish_temperature", true);

  RCLCPP_INFO(get_logger(),
    "Parameters: bus=%s addr=0x%02X frame=%s rate=%.1f Hz mode=%s",
    i2c_bus_.c_str(), i2c_address_, frame_id_.c_str(),
    update_rate_, operation_mode_str_.c_str());
}

// ─── Publisher Setup ─────────────────────────────────────────────────────────

void BNO055Node::createPublishers()
{
  // Main IMU data — QoS: reliable, keep_last(10)
  // This is what robot_localization EKF subscribes to
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "/imu/bno055", rclcpp::SensorDataQoS());

  if (publish_temperature_) {
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      "/imu/bno055/temperature", rclcpp::SensorDataQoS());
  }

  if (publish_diagnostics_) {
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/imu/bno055/status", 10);
  }
}

// ─── Sensor Initialization ──────────────────────────────────────────────────

void BNO055Node::initSensor()
{
  // Parse operation mode string to enum
  OperationMode mode = OperationMode::NDOF;
  if (operation_mode_str_ == "IMU") {
    mode = OperationMode::IMU;
  } else if (operation_mode_str_ == "NDOF") {
    mode = OperationMode::NDOF;
  } else if (operation_mode_str_ == "NDOF_FMC_OFF") {
    mode = OperationMode::NDOF_FMC_OFF;
  } else {
    RCLCPP_WARN(get_logger(),
      "Unknown operation mode '%s', defaulting to NDOF",
      operation_mode_str_.c_str());
  }

  // Create and initialize the I2C driver
  imu_ = std::make_unique<BNO055I2C>(i2c_bus_, static_cast<uint8_t>(i2c_address_));

  if (!imu_->init(mode)) {
    RCLCPP_FATAL(get_logger(),
      "Failed to initialize BNO055 on %s @ 0x%02X. "
      "Check wiring, I2C bus, and run: i2cdetect -y 1",
      i2c_bus_.c_str(), i2c_address_);
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "BNO055 initialized successfully in %s mode",
    operation_mode_str_.c_str());
}

// ─── Timer Setup ─────────────────────────────────────────────────────────────

void BNO055Node::createTimers()
{
  // Main IMU data timer
  auto imu_period = std::chrono::duration<double>(1.0 / update_rate_);
  imu_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(imu_period),
    std::bind(&BNO055Node::imuTimerCallback, this));

  // Diagnostics at 1 Hz (calibration doesn't change fast)
  if (publish_diagnostics_) {
    diag_timer_ = create_wall_timer(
      1s,
      std::bind(&BNO055Node::diagnosticsTimerCallback, this));
  }
}

// ─── IMU Data Callback ──────────────────────────────────────────────────────

void BNO055Node::imuTimerCallback()
{
  if (!imu_ || !imu_->isReady()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "BNO055 not ready, skipping IMU read");
    return;
  }

  // Read all sensor data in one burst
  auto quat = imu_->readQuaternion();
  auto gyro = imu_->readGyroscope();
  auto linear_accel = imu_->readLinearAcceleration();

  // Build the Imu message
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;

  // Orientation quaternion from BNO055 fusion engine
  // BNO055 outputs in ENU convention (when Android orientation mode is selected)
  msg.orientation.w = quat[0];
  msg.orientation.x = quat[1];
  msg.orientation.y = quat[2];
  msg.orientation.z = quat[3];

  // Angular velocity from gyroscope (rad/s)
  msg.angular_velocity.x = gyro[0];
  msg.angular_velocity.y = gyro[1];
  msg.angular_velocity.z = gyro[2];

  // Linear acceleration (gravity-removed) from BNO055 fusion engine
  // This is what robot_localization expects when
  // imu0_remove_gravitational_acceleration is set to true
  msg.linear_acceleration.x = linear_accel[0];
  msg.linear_acceleration.y = linear_accel[1];
  msg.linear_acceleration.z = linear_accel[2];

  // Covariance matrices (diagonal)
  // Using BNO055 datasheet noise specifications
  // Set as row-major 3x3 matrices
  msg.orientation_covariance = {
    ORIENTATION_COVARIANCE, 0.0, 0.0,
    0.0, ORIENTATION_COVARIANCE, 0.0,
    0.0, 0.0, ORIENTATION_COVARIANCE
  };

  msg.angular_velocity_covariance = {
    ANGULAR_VEL_COVARIANCE, 0.0, 0.0,
    0.0, ANGULAR_VEL_COVARIANCE, 0.0,
    0.0, 0.0, ANGULAR_VEL_COVARIANCE
  };

  msg.linear_acceleration_covariance = {
    LINEAR_ACCEL_COVARIANCE, 0.0, 0.0,
    0.0, LINEAR_ACCEL_COVARIANCE, 0.0,
    0.0, 0.0, LINEAR_ACCEL_COVARIANCE
  };

  imu_pub_->publish(msg);

  // Optionally publish temperature (at IMU rate, cheap to read)
  if (publish_temperature_ && temp_pub_) {
    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = msg.header.stamp;
    temp_msg.header.frame_id = frame_id_;
    temp_msg.temperature = static_cast<double>(imu_->readTemperature());
    temp_msg.variance = 1.0;  // ±1°C accuracy per datasheet
    temp_pub_->publish(temp_msg);
  }
}

// ─── Diagnostics Callback ───────────────────────────────────────────────────

void BNO055Node::diagnosticsTimerCallback()
{
  if (!imu_ || !imu_->isReady()) {
    return;
  }

  auto calib = imu_->readCalibrationStatus();
  auto sys_status = imu_->readSystemStatus();
  auto sys_error = imu_->readSystemError();

  auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
  diag_msg.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "BNO055 IMU";
  status.hardware_id = "bno055_i2c";

  // Determine overall status level
  if (calib.isFullyCalibrated() && sys_error == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Fully calibrated, running normally";
  } else if (sys_error != 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "System error code: " + std::to_string(sys_error);
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Calibration incomplete — rotate sensor in figure-8";
  }

  // Add calibration values
  auto addKV = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };

  addKV("calib_sys", std::to_string(calib.sys) + "/3");
  addKV("calib_gyro", std::to_string(calib.gyro) + "/3");
  addKV("calib_accel", std::to_string(calib.accel) + "/3");
  addKV("calib_mag", std::to_string(calib.mag) + "/3");
  addKV("system_status", std::to_string(sys_status));
  addKV("system_error", std::to_string(sys_error));

  diag_msg.status.push_back(status);
  diag_pub_->publish(diag_msg);

  // Log calibration status at INFO level when not fully calibrated
  if (!calib.isFullyCalibrated()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Calibration: sys=%d gyro=%d accel=%d mag=%d — "
      "rotate sensor in figure-8 pattern for magnetometer",
      calib.sys, calib.gyro, calib.accel, calib.mag);
  }
}

}  // namespace bno055_driver
