// Copyright (c) 2026 Juza. MIT License.
// BNO055 I2C Hardware Abstraction Layer
// Provides low-level I2C communication with the BNO055 9-axis IMU.
// Separated from ROS2 logic so it can be unit-tested independently.

#ifndef BNO055_DRIVER__BNO055_I2C_HPP_
#define BNO055_DRIVER__BNO055_I2C_HPP_

#include <cstdint>
#include <string>
#include <array>

namespace bno055_driver
{

// ─── BNO055 Register Map (Page 0) ────────────────────────────────────────────
// Reference: Bosch BNO055 Datasheet BST-BNO055-DS000-14

namespace reg
{
  // Chip identification
  constexpr uint8_t CHIP_ID        = 0x00;  // Should read 0xA0
  constexpr uint8_t ACC_ID         = 0x01;  // Should read 0xFB
  constexpr uint8_t MAG_ID         = 0x02;  // Should read 0x32
  constexpr uint8_t GYR_ID         = 0x03;  // Should read 0x0F

  // Accelerometer data (m/s² when in fusion mode, 1 LSB = 0.01 m/s²)
  constexpr uint8_t ACC_DATA_X_LSB = 0x08;

  // Magnetometer data (1 LSB = 1/16 µT)
  constexpr uint8_t MAG_DATA_X_LSB = 0x0E;

  // Gyroscope data (1 LSB = 1/16 °/s in fusion mode)
  constexpr uint8_t GYR_DATA_X_LSB = 0x14;

  // Euler angles (1 LSB = 1/16 ° or 1/900 rad depending on UNIT_SEL)
  constexpr uint8_t EUL_DATA_H_LSB = 0x1A;

  // Quaternion data (1 LSB = 1/(2^14) = 1/16384)
  constexpr uint8_t QUA_DATA_W_LSB = 0x20;

  // Linear acceleration (gravity-removed, 1 LSB = 0.01 m/s²)
  constexpr uint8_t LIA_DATA_X_LSB = 0x28;

  // Gravity vector (1 LSB = 0.01 m/s²)
  constexpr uint8_t GRV_DATA_X_LSB = 0x2E;

  // Temperature
  constexpr uint8_t TEMP            = 0x34;

  // Calibration status: bits [7:6]=sys, [5:4]=gyr, [3:2]=acc, [1:0]=mag
  constexpr uint8_t CALIB_STAT      = 0x35;

  // System status and error
  constexpr uint8_t SYS_STATUS      = 0x39;
  constexpr uint8_t SYS_ERR         = 0x3A;

  // Unit selection
  constexpr uint8_t UNIT_SEL        = 0x3B;

  // Operation mode
  constexpr uint8_t OPR_MODE        = 0x3D;

  // Power mode
  constexpr uint8_t PWR_MODE        = 0x3E;

  // System trigger (reset, self-test)
  constexpr uint8_t SYS_TRIGGER     = 0x3F;

  // Page ID
  constexpr uint8_t PAGE_ID         = 0x07;

  // Axis remap
  constexpr uint8_t AXIS_MAP_CONFIG = 0x41;
  constexpr uint8_t AXIS_MAP_SIGN   = 0x42;
}  // namespace reg

// ─── Operation Modes ─────────────────────────────────────────────────────────
// Reference: Bosch BNO055 Datasheet BST-BNO055-DS000-14, Section 3.3
//
// Non-fusion modes (raw sensor data only — no quaternion/linear-accel/gravity):
//   CONFIG, ACCONLY, MAGONLY, GYROONLY, ACCMAG, ACCGYRO, MAGGYRO, AMG
//
// Fusion modes (on-chip sensor fusion — provides quaternion, euler, linear-accel, gravity):
//   IMU (IMUPLUS), COMPASS, M4G, NDOF_FMC_OFF, NDOF
//
enum class OperationMode : uint8_t
{
  CONFIG       = 0x00,
  // ─── Non-fusion modes ──────────────────────────────────────────────────
  ACCONLY      = 0x01,  // Accelerometer only
  MAGONLY      = 0x02,  // Magnetometer only
  GYROONLY     = 0x03,  // Gyroscope only
  ACCMAG       = 0x04,  // Accelerometer + Magnetometer
  ACCGYRO      = 0x05,  // Accelerometer + Gyroscope
  MAGGYRO      = 0x06,  // Magnetometer + Gyroscope
  AMG          = 0x07,  // All three sensors, no fusion
  // ─── Fusion modes ─────────────────────────────────────────────────────
  IMU          = 0x08,  // Accel + Gyro relative fusion (no mag) — a.k.a. IMUPLUS
  COMPASS      = 0x09,  // Accel + Mag fusion (heading)
  M4G          = 0x0A,  // Mag for Gyro: gyro replaced by mag for orientation
  NDOF_FMC_OFF = 0x0B,  // Full 9-axis fusion without fast mag calibration
  NDOF         = 0x0C,  // Full 9-axis fusion (recommended for SLAM)
};

/// @brief Check whether a mode is a fusion mode.
/// Fusion modes run the BNO055 on-chip sensor fusion engine and produce
/// quaternion, Euler angles, linear acceleration, and gravity vector outputs.
inline bool isFusionMode(OperationMode mode)
{
  return static_cast<uint8_t>(mode) >= static_cast<uint8_t>(OperationMode::IMU);
}

/// @brief Check whether a mode uses the magnetometer.
inline bool usesMagnetometer(OperationMode mode)
{
  switch (mode) {
    case OperationMode::MAGONLY:
    case OperationMode::ACCMAG:
    case OperationMode::MAGGYRO:
    case OperationMode::AMG:
    case OperationMode::COMPASS:
    case OperationMode::M4G:
    case OperationMode::NDOF_FMC_OFF:
    case OperationMode::NDOF:
      return true;
    default:
      return false;
  }
}

/// @brief Check whether a mode uses the gyroscope.
/// Note: M4G uses magnetometer to replace gyro, so gyro is NOT used in M4G.
inline bool usesGyroscope(OperationMode mode)
{
  switch (mode) {
    case OperationMode::GYROONLY:
    case OperationMode::ACCGYRO:
    case OperationMode::MAGGYRO:
    case OperationMode::AMG:
    case OperationMode::IMU:
    case OperationMode::NDOF_FMC_OFF:
    case OperationMode::NDOF:
      return true;
    default:
      return false;
  }
}

/// @brief Check whether a mode uses the accelerometer.
inline bool usesAccelerometer(OperationMode mode)
{
  switch (mode) {
    case OperationMode::ACCONLY:
    case OperationMode::ACCMAG:
    case OperationMode::ACCGYRO:
    case OperationMode::AMG:
    case OperationMode::IMU:
    case OperationMode::COMPASS:
    case OperationMode::M4G:
    case OperationMode::NDOF_FMC_OFF:
    case OperationMode::NDOF:
      return true;
    default:
      return false;
  }
}

// ─── Power Modes ─────────────────────────────────────────────────────────────
enum class PowerMode : uint8_t
{
  NORMAL  = 0x00,
  LOW     = 0x01,
  SUSPEND = 0x02,
};

// ─── Axis Placement Positions (P0–P7) ────────────────────────────────────────
// These map to the 8 possible physical orientations of the BNO055 chip.
// Reference: Bosch BNO055 Datasheet, Section 3.4 "Axis Remap"
//
//   P0 — Chip facing up, connector at right  (default at POR)
//   P1 — Chip facing up, connector at left   (BNO055 default after config)
//   P2 — Chip facing up, connector at bottom
//   P3 — Chip facing up, connector at top
//   P4 — Chip facing DOWN (upside-down mounting)
//   P5 — Rotated 90° CW
//   P6 — Rotated 180°
//   P7 — Rotated 270° CW
//
enum class AxisPlacement : uint8_t
{
  P0 = 0,
  P1 = 1,
  P2 = 2,
  P3 = 3,
  P4 = 4,
  P5 = 5,
  P6 = 6,
  P7 = 7,
};

// Axis remap register values for each placement.
// AXIS_MAP_CONFIG (0x41) sets which physical axis maps to X/Y/Z.
// AXIS_MAP_SIGN  (0x42) sets the sign (positive/negative) for each axis.
struct AxisRemapConfig
{
  uint8_t config;  // Value for AXIS_MAP_CONFIG register (0x41)
  uint8_t sign;    // Value for AXIS_MAP_SIGN register   (0x42)
};

// ─── Calibration Status ──────────────────────────────────────────────────────
struct CalibrationStatus
{
  uint8_t sys;   // 0–3
  uint8_t gyro;  // 0–3
  uint8_t accel; // 0–3
  uint8_t mag;   // 0–3

  bool isFullyCalibrated() const
  {
    return sys == 3 && gyro == 3 && accel == 3 && mag == 3;
  }

  /// IMU mode doesn't use the magnetometer, so ignore mag calibration.
  bool isFullyCalibratedIMU() const
  {
    return sys == 3 && gyro == 3 && accel == 3;
  }
};

// ─── BNO055 I2C Driver Class ─────────────────────────────────────────────────
class BNO055I2C
{
public:
  /// @brief Construct the I2C driver.
  /// @param bus I2C device path, e.g. "/dev/i2c-1"
  /// @param address 7-bit I2C address (0x28 or 0x29)
  BNO055I2C(const std::string & bus, uint8_t address);

  ~BNO055I2C();

  // Prevent copy
  BNO055I2C(const BNO055I2C &) = delete;
  BNO055I2C & operator=(const BNO055I2C &) = delete;

  /// @brief Initialize the sensor: verify chip ID, reset, configure units, set mode.
  /// @param mode Operation mode (default NDOF for full fusion)
  /// @param placement Axis placement position (default P1 — BNO055 chip default)
  /// @return true on success
  bool init(OperationMode mode = OperationMode::NDOF,
            AxisPlacement placement = AxisPlacement::P1);

  /// @brief Check if the I2C bus is open and chip ID is valid.
  bool isReady() const;

  /// @brief Set the operation mode.
  /// @note Must go through CONFIG mode first (handled internally).
  bool setOperationMode(OperationMode mode);

  /// @brief Configure axis remap for non-standard sensor mounting.
  /// @param placement One of P0–P7 physical placement positions.
  /// @return true on success
  /// @note Must be called while in CONFIG mode (handled internally).
  bool setAxisRemap(AxisPlacement placement);

  // ─── Data reading methods ──────────────────────────────────────────────

  /// @brief Read accelerometer data.
  /// @return {ax, ay, az} in m/s²
  /// @note Available in all modes that use the accelerometer.
  std::array<double, 3> readAccelerometer();

  /// @brief Read magnetometer data.
  /// @return {mx, my, mz} in µT (microtesla)
  /// @note Available in all modes that use the magnetometer.
  std::array<double, 3> readMagnetometer();

  /// @brief Read gyroscope data.
  /// @return {gx, gy, gz} in rad/s
  /// @note Available in all modes that use the gyroscope.
  std::array<double, 3> readGyroscope();

  /// @brief Read Euler angles from fusion engine.
  /// @return {heading, roll, pitch} in radians (when UNIT_SEL configured for radians)
  /// @note Only available in fusion modes.
  std::array<double, 3> readEuler();

  /// @brief Read quaternion orientation from fusion engine.
  /// @return {w, x, y, z} normalized quaternion
  /// @note Only available in fusion modes.
  std::array<double, 4> readQuaternion();

  /// @brief Read linear acceleration (gravity removed).
  /// @return {ax, ay, az} in m/s²
  /// @note Only available in fusion modes.
  std::array<double, 3> readLinearAcceleration();

  /// @brief Read gravity vector.
  /// @return {gx, gy, gz} in m/s²
  /// @note Only available in fusion modes.
  std::array<double, 3> readGravityVector();

  /// @brief Read chip temperature.
  /// @return temperature in °C
  int8_t readTemperature();

  /// @brief Read calibration status for all subsystems.
  CalibrationStatus readCalibrationStatus();

  /// @brief Read system status register.
  uint8_t readSystemStatus();

  /// @brief Read system error register.
  uint8_t readSystemError();

private:
  // ─── Low-level I2C primitives ──────────────────────────────────────────

  /// @brief Write a single byte to a register.
  bool writeRegister(uint8_t reg, uint8_t value);

  /// @brief Read a single byte from a register.
  bool readRegister(uint8_t reg, uint8_t & value);

  /// @brief Burst-read multiple bytes starting from a register address.
  /// Uses I2C_RDWR ioctl for repeated-start (required by BNO055).
  bool readRegisters(uint8_t start_reg, uint8_t * buffer, uint8_t length);

  /// @brief Read a signed 16-bit value (little-endian) from two consecutive registers.
  int16_t readS16(uint8_t reg);

  /// @brief Read 3-axis signed 16-bit data (6 bytes) starting from a register.
  std::array<int16_t, 3> readVector3(uint8_t start_reg);

  /// @brief Read 4-component signed 16-bit data (8 bytes) starting from a register.
  std::array<int16_t, 4> readVector4(uint8_t start_reg);

  // ─── Members ───────────────────────────────────────────────────────────
  std::string bus_path_;
  uint8_t address_;
  int fd_;           // File descriptor for I2C bus
  bool ready_;
  OperationMode current_mode_;
};

}  // namespace bno055_driver

#endif  // BNO055_DRIVER__BNO055_I2C_HPP_
