// Copyright (c) 2026 Juza. MIT License.
// BNO055 ROS2 Node Implementation
//
// Polls the BNO055 over I2C at a configurable rate and publishes:
//   - sensor_msgs/Imu           on /imu/bno055           (orientation + gyro + accel)
//   - sensor_msgs/MagneticField on /imu/bno055/mag        (magnetometer data)
//   - sensor_msgs/Temperature   on /imu/bno055/temperature
//   - diagnostic_msgs/DiagnosticArray on /imu/bno055/status (calibration)
//
// The node adapts its published data based on the selected BNO055 operation mode:
//   - Fusion modes (IMU/IMUPLUS, COMPASS, M4G, NDOF, NDOF_FMC_OFF):
//       Publishes fused orientation quaternion, linear acceleration, angular velocity.
//   - Non-fusion modes (ACCONLY, MAGONLY, GYROONLY, ACCMAG, ACCGYRO, MAGGYRO, AMG):
//       Publishes raw sensor data only. Orientation covariance is set to -1 to
//       indicate "unknown" orientation per the sensor_msgs/Imu specification.
//
// Designed for integration with robot_localization EKF for SLAM mapping.

#include "bno055_driver/bno055_node.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace bno055_driver
{

// ─── Mode string ↔ enum conversion ───────────────────────────────────────────

OperationMode BNO055Node::operationModeFromString(const std::string & mode_str)
{
  if (mode_str == "ACCONLY")       return OperationMode::ACCONLY;
  if (mode_str == "MAGONLY")       return OperationMode::MAGONLY;
  if (mode_str == "GYROONLY")      return OperationMode::GYROONLY;
  if (mode_str == "ACCMAG")        return OperationMode::ACCMAG;
  if (mode_str == "ACCGYRO")       return OperationMode::ACCGYRO;
  if (mode_str == "MAGGYRO")       return OperationMode::MAGGYRO;
  if (mode_str == "AMG")           return OperationMode::AMG;
  if (mode_str == "IMU" || mode_str == "IMUPLUS")
                                   return OperationMode::IMU;
  if (mode_str == "COMPASS")       return OperationMode::COMPASS;
  if (mode_str == "M4G")           return OperationMode::M4G;
  if (mode_str == "NDOF_FMC_OFF")  return OperationMode::NDOF_FMC_OFF;
  if (mode_str == "NDOF")          return OperationMode::NDOF;
  // Unknown — return NDOF as a safe default
  return OperationMode::NDOF;
}

std::string BNO055Node::operationModeToString(OperationMode mode)
{
  switch (mode) {
    case OperationMode::CONFIG:       return "CONFIG";
    case OperationMode::ACCONLY:      return "ACCONLY";
    case OperationMode::MAGONLY:      return "MAGONLY";
    case OperationMode::GYROONLY:     return "GYROONLY";
    case OperationMode::ACCMAG:       return "ACCMAG";
    case OperationMode::ACCGYRO:      return "ACCGYRO";
    case OperationMode::MAGGYRO:      return "MAGGYRO";
    case OperationMode::AMG:          return "AMG";
    case OperationMode::IMU:          return "IMU (IMUPLUS)";
    case OperationMode::COMPASS:      return "COMPASS";
    case OperationMode::M4G:          return "M4G";
    case OperationMode::NDOF_FMC_OFF: return "NDOF_FMC_OFF";
    case OperationMode::NDOF:         return "NDOF";
    default:                          return "UNKNOWN";
  }
}

// ─── Constructor ─────────────────────────────────────────────────────────────

BNO055Node::BNO055Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("bno055_node", options),
  operation_mode_(OperationMode::NDOF)
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
  publish_magnetometer_ = declare_parameter<bool>("publish_magnetometer", true);

  // Axis remap for non-standard mounting
  placement_axis_remap_str_ = declare_parameter<std::string>("placement_axis_remap", "P1");

  // Parse the operation mode string early so we can log it
  operation_mode_ = operationModeFromString(operation_mode_str_);

  // Validate mode string
  const auto known_modes = {
    "ACCONLY", "MAGONLY", "GYROONLY", "ACCMAG", "ACCGYRO", "MAGGYRO", "AMG",
    "IMU", "IMUPLUS", "COMPASS", "M4G", "NDOF_FMC_OFF", "NDOF"
  };
  bool mode_found = false;
  for (const auto & m : known_modes) {
    if (operation_mode_str_ == m) { mode_found = true; break; }
  }
  if (!mode_found) {
    RCLCPP_WARN(get_logger(),
      "Unknown operation mode '%s', defaulting to NDOF. "
      "Valid modes: ACCONLY, MAGONLY, GYROONLY, ACCMAG, ACCGYRO, MAGGYRO, AMG, "
      "IMU (IMUPLUS), COMPASS, M4G, NDOF_FMC_OFF, NDOF",
      operation_mode_str_.c_str());
    operation_mode_str_ = "NDOF";
    operation_mode_ = OperationMode::NDOF;
  }

  RCLCPP_INFO(get_logger(),
    "Parameters: bus=%s addr=0x%02X frame=%s rate=%.1f Hz mode=%s placement=%s",
    i2c_bus_.c_str(), i2c_address_, frame_id_.c_str(),
    update_rate_, operationModeToString(operation_mode_).c_str(),
    placement_axis_remap_str_.c_str());

  // Log mode characteristics
  if (isFusionMode(operation_mode_)) {
    RCLCPP_INFO(get_logger(),
      "Fusion mode selected — quaternion, euler, linear-accel, gravity data available");
  } else {
    RCLCPP_INFO(get_logger(),
      "Non-fusion mode selected — raw sensor data only (no orientation fusion)");
  }
  if (usesMagnetometer(operation_mode_)) {
    RCLCPP_INFO(get_logger(), "Mode uses magnetometer — mag data will be published");
  }
}

// ─── Publisher Setup ─────────────────────────────────────────────────────────

void BNO055Node::createPublishers()
{
  // Main IMU data — QoS: reliable, keep_last(10)
  // This is what robot_localization EKF subscribes to
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "/imu/bno055", rclcpp::SensorDataQoS());

  // Magnetometer publisher — only if mode uses mag and user wants it
  if (publish_magnetometer_ && usesMagnetometer(operation_mode_)) {
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
      "/imu/bno055/mag", rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(), "Magnetometer publisher enabled on /imu/bno055/mag");
  }

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
  // Parse axis placement string to enum
  AxisPlacement placement = AxisPlacement::P1;  // Default: BNO055 standard
  if (placement_axis_remap_str_.size() == 2 &&
      (placement_axis_remap_str_[0] == 'P' || placement_axis_remap_str_[0] == 'p')) {
    int p = placement_axis_remap_str_[1] - '0';
    if (p >= 0 && p <= 7) {
      placement = static_cast<AxisPlacement>(p);
    } else {
      RCLCPP_WARN(get_logger(),
        "Invalid placement_axis_remap '%s' (must be P0-P7), defaulting to P1",
        placement_axis_remap_str_.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(),
      "Invalid placement_axis_remap format '%s' (expected P0-P7), defaulting to P1",
      placement_axis_remap_str_.c_str());
  }

  // Create and initialize the I2C driver
  imu_ = std::make_unique<BNO055I2C>(i2c_bus_, static_cast<uint8_t>(i2c_address_));

  if (!imu_->init(operation_mode_, placement)) {
    RCLCPP_FATAL(get_logger(),
      "Failed to initialize BNO055 on %s @ 0x%02X. "
      "Check wiring, I2C bus, and run: i2cdetect -y 1",
      i2c_bus_.c_str(), i2c_address_);
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "BNO055 initialized successfully in %s mode (placement %s)",
    operationModeToString(operation_mode_).c_str(), placement_axis_remap_str_.c_str());
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

  auto stamp = now();

  // Build the Imu message
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;

  // ── Orientation ─────────────────────────────────────────────────────────
  if (isFusionMode(operation_mode_)) {
    // Fusion modes: read quaternion from the on-chip fusion engine
    auto quat = imu_->readQuaternion();
    msg.orientation.w = quat[0];
    msg.orientation.x = quat[1];
    msg.orientation.y = quat[2];
    msg.orientation.z = quat[3];

    msg.orientation_covariance = {
      ORIENTATION_COVARIANCE, 0.0, 0.0,
      0.0, ORIENTATION_COVARIANCE, 0.0,
      0.0, 0.0, ORIENTATION_COVARIANCE
    };
  } else {
    // Non-fusion modes: orientation is not available
    // Per sensor_msgs/Imu spec: set covariance[0] = -1 to indicate "unknown"
    msg.orientation.w = 0.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation_covariance[0] = -1.0;
  }

  // ── Angular Velocity ────────────────────────────────────────────────────
  if (usesGyroscope(operation_mode_)) {
    auto gyro = imu_->readGyroscope();
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];

    msg.angular_velocity_covariance = {
      ANGULAR_VEL_COVARIANCE, 0.0, 0.0,
      0.0, ANGULAR_VEL_COVARIANCE, 0.0,
      0.0, 0.0, ANGULAR_VEL_COVARIANCE
    };
  } else {
    // Gyro not available — set covariance[0] = -1
    msg.angular_velocity_covariance[0] = -1.0;
  }

  // ── Linear Acceleration ─────────────────────────────────────────────────
  if (isFusionMode(operation_mode_)) {
    // Fusion modes: use gravity-removed linear acceleration
    auto linear_accel = imu_->readLinearAcceleration();
    msg.linear_acceleration.x = linear_accel[0];
    msg.linear_acceleration.y = linear_accel[1];
    msg.linear_acceleration.z = linear_accel[2];
  } else if (usesAccelerometer(operation_mode_)) {
    // Non-fusion modes: use raw accelerometer (includes gravity)
    auto accel = imu_->readAccelerometer();
    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];
  } else {
    // No accelerometer in this mode — set covariance[0] = -1
    msg.linear_acceleration_covariance[0] = -1.0;
  }

  if (usesAccelerometer(operation_mode_)) {
    msg.linear_acceleration_covariance = {
      LINEAR_ACCEL_COVARIANCE, 0.0, 0.0,
      0.0, LINEAR_ACCEL_COVARIANCE, 0.0,
      0.0, 0.0, LINEAR_ACCEL_COVARIANCE
    };
  }

  imu_pub_->publish(msg);

  // ── Magnetometer ────────────────────────────────────────────────────────
  if (mag_pub_ && usesMagnetometer(operation_mode_)) {
    auto mag_data = imu_->readMagnetometer();
    auto mag_msg = sensor_msgs::msg::MagneticField();
    mag_msg.header.stamp = stamp;
    mag_msg.header.frame_id = frame_id_;

    // BNO055 outputs µT, sensor_msgs/MagneticField expects Tesla
    // 1 µT = 1e-6 T
    mag_msg.magnetic_field.x = mag_data[0] * 1e-6;
    mag_msg.magnetic_field.y = mag_data[1] * 1e-6;
    mag_msg.magnetic_field.z = mag_data[2] * 1e-6;

    mag_msg.magnetic_field_covariance = {
      MAGNETIC_FIELD_COVARIANCE * 1e-12, 0.0, 0.0,
      0.0, MAGNETIC_FIELD_COVARIANCE * 1e-12, 0.0,
      0.0, 0.0, MAGNETIC_FIELD_COVARIANCE * 1e-12
    };

    mag_pub_->publish(mag_msg);
  }

  // ── Temperature ─────────────────────────────────────────────────────────
  if (publish_temperature_ && temp_pub_) {
    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = stamp;
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

  // Determine overall status level — mode-aware calibration check
  bool fullyCalibrated = false;
  if (!isFusionMode(operation_mode_)) {
    // Non-fusion modes: calibration of individual sensors
    bool ok = true;
    if (usesAccelerometer(operation_mode_) && calib.accel < 3) ok = false;
    if (usesGyroscope(operation_mode_) && calib.gyro < 3) ok = false;
    if (usesMagnetometer(operation_mode_) && calib.mag < 3) ok = false;
    fullyCalibrated = ok;
  } else if (!usesMagnetometer(operation_mode_)) {
    // Fusion modes without magnetometer (IMU/IMUPLUS)
    fullyCalibrated = calib.isFullyCalibratedIMU();
  } else {
    // Fusion modes with magnetometer (COMPASS, M4G, NDOF, NDOF_FMC_OFF)
    fullyCalibrated = calib.isFullyCalibrated();
  }

  if (fullyCalibrated && sys_error == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Fully calibrated, running normally in " +
      operationModeToString(operation_mode_) + " mode";
  } else if (sys_error != 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "System error code: " + std::to_string(sys_error);
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;

    // Mode-aware calibration guidance
    if (!usesMagnetometer(operation_mode_)) {
      // Modes without magnetometer (IMU, ACCONLY, GYROONLY, ACCGYRO)
      if (usesGyroscope(operation_mode_) && calib.gyro < 3) {
        status.message = "Calibration incomplete — hold sensor still for gyroscope";
      } else if (isFusionMode(operation_mode_) && calib.sys < 3) {
        status.message = "Sensors ready — waiting for fusion engine (sys) to converge, hold still";
      } else if (usesAccelerometer(operation_mode_) && calib.accel < 3) {
        status.message = "Calibration incomplete — slowly rotate sensor for accelerometer";
      } else {
        status.message = "Calibration incomplete";
      }
    } else {
      // Modes with magnetometer (COMPASS, M4G, NDOF, MAGONLY, ACCMAG, MAGGYRO, AMG)
      if (calib.mag < 3) {
        status.message = "Calibration incomplete — rotate sensor in figure-8 for magnetometer";
      } else if (usesGyroscope(operation_mode_) && calib.gyro < 3) {
        status.message = "Calibration incomplete — hold sensor still for gyroscope";
      } else if (isFusionMode(operation_mode_) && calib.sys < 3) {
        status.message = "Sensors ready — waiting for fusion engine (sys) to converge";
      } else {
        status.message = "Calibration incomplete";
      }
    }
  }

  // Add calibration values
  auto addKV = [&](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };

  addKV("operation_mode", operationModeToString(operation_mode_));
  addKV("fusion_mode", isFusionMode(operation_mode_) ? "true" : "false");
  addKV("calib_sys", std::to_string(calib.sys) + "/3");
  addKV("calib_gyro", std::to_string(calib.gyro) + "/3");
  addKV("calib_accel", std::to_string(calib.accel) + "/3");
  addKV("calib_mag", std::to_string(calib.mag) + "/3");
  addKV("system_status", std::to_string(sys_status));
  addKV("system_error", std::to_string(sys_error));

  diag_msg.status.push_back(status);
  diag_pub_->publish(diag_msg);

  // Log calibration status at INFO level when not fully calibrated
  if (!fullyCalibrated) {
    if (usesMagnetometer(operation_mode_)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "[%s] Calibration: sys=%d gyro=%d accel=%d mag=%d — %s",
        operationModeToString(operation_mode_).c_str(),
        calib.sys, calib.gyro, calib.accel, calib.mag,
        calib.mag < 3 ? "rotate sensor in figure-8 for magnetometer" :
        calib.sys < 3 ? "hold still, waiting for fusion engine" : "calibrating");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "[%s] Calibration: sys=%d gyro=%d accel=%d (magnetometer not used)",
        operationModeToString(operation_mode_).c_str(),
        calib.sys, calib.gyro, calib.accel);
    }
  }
}

}  // namespace bno055_driver
