// Copyright (c) 2026 Juza. MIT License.
// BNO055 I2C Hardware Abstraction Layer — Implementation
//
// Uses Linux i2c-dev interface with I2C_RDWR ioctl for reliable
// repeated-start transactions (required by BNO055 for register reads).
//
// RPi4 note: If you get I2C errors, add dtoverlay=i2c-gpio to
// /boot/firmware/config.txt for software I2C with clock-stretching support.

#include "bno055_driver/bno055_i2c.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <cstring>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>

namespace bno055_driver
{

// ─── Expected chip IDs ───────────────────────────────────────────────────────
static constexpr uint8_t BNO055_CHIP_ID_VALUE = 0xA0;

// ─── Timing constants (from datasheet) ───────────────────────────────────────
// After POR/reset: wait at least 650ms before first I2C transaction
static constexpr int RESET_DELAY_MS = 700;
// After mode switch: CONFIG→any = 7ms, any→CONFIG = 19ms
static constexpr int MODE_SWITCH_DELAY_MS = 25;

// ─── Axis remap lookup table (P0–P7) ─────────────────────────────────────────
// From BNO055 datasheet, Section 3.4, Table 3-4
//   AXIS_MAP_CONFIG (0x41): bits [5:4]=Z axis, [3:2]=Y axis, [1:0]=X axis
//     Axis codes: 0=X, 1=Y, 2=Z
//   AXIS_MAP_SIGN  (0x42): bit 2=X sign, bit 1=Y sign, bit 0=Z sign
//     Sign bits: 0=positive, 1=negative
static constexpr AxisRemapConfig PLACEMENT_CONFIGS[] = {
  {0x21, 0x04},  // P0: X=X, Y=Y, Z=Z, X-neg         (chip up, connector right)
  {0x24, 0x00},  // P1: X=X, Y=Y, Z=Z, all positive   (BNO055 default)
  {0x24, 0x06},  // P2: X=X, Y=Y, Z=Z, X-neg Y-neg    (chip up, connector down)
  {0x21, 0x02},  // P3: X=X, Y=Y, Z=Z, Y-neg           (chip up, connector up)
  {0x24, 0x03},  // P4: X=X, Y=Y, Z=Z, Y-neg Z-neg    (chip UPSIDE DOWN)
  {0x21, 0x01},  // P5: X→Y, Y→X, Z=Z, Z-neg           (rotated 90°)
  {0x21, 0x07},  // P6: X=X, Y=Y, Z=Z, all neg         (rotated 180°)
  {0x24, 0x05},  // P7: X→Y, Y→X, Z=Z, X-neg Z-neg    (rotated 270°)
};

// ─── Constructor / Destructor ────────────────────────────────────────────────

BNO055I2C::BNO055I2C(const std::string & bus, uint8_t address)
: bus_path_(bus),
  address_(address),
  fd_(-1),
  ready_(false),
  current_mode_(OperationMode::CONFIG)
{
}

BNO055I2C::~BNO055I2C()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

// ─── Initialization ──────────────────────────────────────────────────────────

bool BNO055I2C::init(OperationMode mode, AxisPlacement placement)
{
  // Open the I2C bus
  fd_ = open(bus_path_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "[BNO055] Failed to open I2C bus: " << bus_path_
              << " (errno: " << strerror(errno) << ")" << std::endl;
    return false;
  }

  // Verify chip ID
  uint8_t chip_id = 0;
  if (!readRegister(reg::CHIP_ID, chip_id) || chip_id != BNO055_CHIP_ID_VALUE) {
    std::cerr << "[BNO055] Chip ID mismatch. Expected 0xA0, got 0x"
              << std::hex << static_cast<int>(chip_id) << std::dec << std::endl;
    close(fd_);
    fd_ = -1;
    return false;
  }

  // Switch to CONFIG mode for setup
  if (!writeRegister(reg::OPR_MODE, static_cast<uint8_t>(OperationMode::CONFIG))) {
    std::cerr << "[BNO055] Failed to enter CONFIG mode" << std::endl;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(MODE_SWITCH_DELAY_MS));
  current_mode_ = OperationMode::CONFIG;

  // Trigger a system reset for a clean start
  if (!writeRegister(reg::SYS_TRIGGER, 0x20)) {
    std::cerr << "[BNO055] Failed to trigger reset" << std::endl;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(RESET_DELAY_MS));

  // Wait for chip to come back — poll chip ID
  for (int retry = 0; retry < 10; ++retry) {
    chip_id = 0;
    if (readRegister(reg::CHIP_ID, chip_id) && chip_id == BNO055_CHIP_ID_VALUE) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (chip_id != BNO055_CHIP_ID_VALUE) {
    std::cerr << "[BNO055] Sensor did not respond after reset" << std::endl;
    return false;
  }

  // Set power mode to NORMAL
  if (!writeRegister(reg::PWR_MODE, static_cast<uint8_t>(PowerMode::NORMAL))) {
    std::cerr << "[BNO055] Failed to set power mode" << std::endl;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Select page 0
  if (!writeRegister(reg::PAGE_ID, 0x00)) {
    std::cerr << "[BNO055] Failed to select page 0" << std::endl;
    return false;
  }

  // Configure units:
  //  bit 0: acceleration = m/s²  (0)
  //  bit 1: angular rate = rad/s (1)  ← critical for ROS2 sensor_msgs/Imu
  //  bit 2: euler angles = radians (1)
  //  bit 4: temperature = °C (0)
  //  bit 7: orientation = Android (0) → ENU-compatible
  // Value: 0b00000110 = 0x06
  // NOTE: We set gyro to rad/s (bit 1) and euler to radians (bit 2)
  //       but keep accel in m/s² (bit 0 = 0)
  if (!writeRegister(reg::UNIT_SEL, 0x06)) {
    std::cerr << "[BNO055] Failed to configure units" << std::endl;
    return false;
  }

  // Clear system trigger register
  if (!writeRegister(reg::SYS_TRIGGER, 0x00)) {
    std::cerr << "[BNO055] Failed to clear SYS_TRIGGER" << std::endl;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Apply axis remap if not the default P1
  if (placement != AxisPlacement::P1) {
    if (!setAxisRemap(placement)) {
      std::cerr << "[BNO055] Failed to set axis remap for placement P"
                << static_cast<int>(placement) << std::endl;
      return false;
    }
    std::cout << "[BNO055] Axis remap set to P"
              << static_cast<int>(placement) << std::endl;
  }

  // Set the requested operation mode (e.g., NDOF for full 9-axis fusion)
  if (!setOperationMode(mode)) {
    std::cerr << "[BNO055] Failed to set operation mode" << std::endl;
    return false;
  }

  ready_ = true;
  std::cout << "[BNO055] Initialized successfully on " << bus_path_
            << " @ 0x" << std::hex << static_cast<int>(address_) << std::dec
            << " in mode 0x" << std::hex << static_cast<int>(mode) << std::dec
            << std::endl;
  return true;
}

bool BNO055I2C::isReady() const
{
  return ready_ && fd_ >= 0;
}

bool BNO055I2C::setOperationMode(OperationMode mode)
{
  // Must transition through CONFIG mode if not already there
  if (current_mode_ != OperationMode::CONFIG && mode != OperationMode::CONFIG) {
    if (!writeRegister(reg::OPR_MODE, static_cast<uint8_t>(OperationMode::CONFIG))) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(MODE_SWITCH_DELAY_MS));
  }

  if (!writeRegister(reg::OPR_MODE, static_cast<uint8_t>(mode))) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(MODE_SWITCH_DELAY_MS));

  current_mode_ = mode;
  return true;
}

// ─── Axis Remap ──────────────────────────────────────────────────────────────

bool BNO055I2C::setAxisRemap(AxisPlacement placement)
{
  uint8_t idx = static_cast<uint8_t>(placement);
  if (idx > 7) {
    std::cerr << "[BNO055] Invalid placement index: " << static_cast<int>(idx) << std::endl;
    return false;
  }

  // Must be in CONFIG mode to write axis remap registers
  if (current_mode_ != OperationMode::CONFIG) {
    if (!writeRegister(reg::OPR_MODE, static_cast<uint8_t>(OperationMode::CONFIG))) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(MODE_SWITCH_DELAY_MS));
  }

  const auto & remap = PLACEMENT_CONFIGS[idx];

  if (!writeRegister(reg::AXIS_MAP_CONFIG, remap.config)) {
    std::cerr << "[BNO055] Failed to write AXIS_MAP_CONFIG" << std::endl;
    return false;
  }

  if (!writeRegister(reg::AXIS_MAP_SIGN, remap.sign)) {
    std::cerr << "[BNO055] Failed to write AXIS_MAP_SIGN" << std::endl;
    return false;
  }

  // Verify the write
  uint8_t read_config = 0, read_sign = 0;
  readRegister(reg::AXIS_MAP_CONFIG, read_config);
  readRegister(reg::AXIS_MAP_SIGN, read_sign);

  if (read_config != remap.config || read_sign != remap.sign) {
    std::cerr << "[BNO055] Axis remap verification failed. "
              << "Expected config=0x" << std::hex << static_cast<int>(remap.config)
              << " sign=0x" << static_cast<int>(remap.sign)
              << ", got config=0x" << static_cast<int>(read_config)
              << " sign=0x" << static_cast<int>(read_sign) << std::dec << std::endl;
    return false;
  }

  return true;
}

// ─── Data Reading Methods ────────────────────────────────────────────────────

std::array<double, 3> BNO055I2C::readAccelerometer()
{
  // In fusion mode with m/s² units: 1 LSB = 0.01 m/s²
  auto raw = readVector3(reg::ACC_DATA_X_LSB);
  return {
    static_cast<double>(raw[0]) / 100.0,
    static_cast<double>(raw[1]) / 100.0,
    static_cast<double>(raw[2]) / 100.0
  };
}

std::array<double, 3> BNO055I2C::readGyroscope()
{
  // With UNIT_SEL bit 1 set (rad/s): 1 LSB = 1/900 rad/s
  auto raw = readVector3(reg::GYR_DATA_X_LSB);
  return {
    static_cast<double>(raw[0]) / 900.0,
    static_cast<double>(raw[1]) / 900.0,
    static_cast<double>(raw[2]) / 900.0
  };
}

std::array<double, 4> BNO055I2C::readQuaternion()
{
  // Quaternion: 1 LSB = 1/(2^14) = 1/16384
  auto raw = readVector4(reg::QUA_DATA_W_LSB);
  constexpr double scale = 1.0 / 16384.0;
  return {
    static_cast<double>(raw[0]) * scale,  // w
    static_cast<double>(raw[1]) * scale,  // x
    static_cast<double>(raw[2]) * scale,  // y
    static_cast<double>(raw[3]) * scale   // z
  };
}

std::array<double, 3> BNO055I2C::readLinearAcceleration()
{
  // Gravity-removed acceleration: 1 LSB = 0.01 m/s²
  auto raw = readVector3(reg::LIA_DATA_X_LSB);
  return {
    static_cast<double>(raw[0]) / 100.0,
    static_cast<double>(raw[1]) / 100.0,
    static_cast<double>(raw[2]) / 100.0
  };
}

std::array<double, 3> BNO055I2C::readGravityVector()
{
  auto raw = readVector3(reg::GRV_DATA_X_LSB);
  return {
    static_cast<double>(raw[0]) / 100.0,
    static_cast<double>(raw[1]) / 100.0,
    static_cast<double>(raw[2]) / 100.0
  };
}

int8_t BNO055I2C::readTemperature()
{
  uint8_t value = 0;
  readRegister(reg::TEMP, value);
  return static_cast<int8_t>(value);
}

CalibrationStatus BNO055I2C::readCalibrationStatus()
{
  uint8_t value = 0;
  readRegister(reg::CALIB_STAT, value);
  return {
    static_cast<uint8_t>((value >> 6) & 0x03),  // sys
    static_cast<uint8_t>((value >> 4) & 0x03),  // gyro
    static_cast<uint8_t>((value >> 2) & 0x03),  // accel
    static_cast<uint8_t>((value >> 0) & 0x03)   // mag
  };
}

uint8_t BNO055I2C::readSystemStatus()
{
  uint8_t value = 0;
  readRegister(reg::SYS_STATUS, value);
  return value;
}

uint8_t BNO055I2C::readSystemError()
{
  uint8_t value = 0;
  readRegister(reg::SYS_ERR, value);
  return value;
}

// ─── Low-level I2C Primitives ────────────────────────────────────────────────

bool BNO055I2C::writeRegister(uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};

  struct i2c_msg msg;
  msg.addr = address_;
  msg.flags = 0;  // write
  msg.len = 2;
  msg.buf = buf;

  struct i2c_rdwr_ioctl_data data;
  data.msgs = &msg;
  data.nmsgs = 1;

  if (ioctl(fd_, I2C_RDWR, &data) < 0) {
    std::cerr << "[BNO055] I2C write failed for register 0x"
              << std::hex << static_cast<int>(reg) << std::dec
              << ": " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

bool BNO055I2C::readRegister(uint8_t reg, uint8_t & value)
{
  return readRegisters(reg, &value, 1);
}

bool BNO055I2C::readRegisters(uint8_t start_reg, uint8_t * buffer, uint8_t length)
{
  // Uses I2C_RDWR for a repeated-start transaction:
  // [START][ADDR+W][REG][REPEATED-START][ADDR+R][DATA...][STOP]
  // This is critical for BNO055 — simple write-then-read without
  // repeated-start can fail on some I2C controllers.

  struct i2c_msg messages[2];

  // Message 1: Write the register address
  messages[0].addr = address_;
  messages[0].flags = 0;  // write
  messages[0].len = 1;
  messages[0].buf = &start_reg;

  // Message 2: Read the data
  messages[1].addr = address_;
  messages[1].flags = I2C_M_RD;  // read
  messages[1].len = length;
  messages[1].buf = buffer;

  struct i2c_rdwr_ioctl_data data;
  data.msgs = messages;
  data.nmsgs = 2;

  if (ioctl(fd_, I2C_RDWR, &data) < 0) {
    std::cerr << "[BNO055] I2C read failed for register 0x"
              << std::hex << static_cast<int>(start_reg) << std::dec
              << " (len=" << static_cast<int>(length) << ")"
              << ": " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

int16_t BNO055I2C::readS16(uint8_t reg)
{
  uint8_t buf[2] = {0, 0};
  readRegisters(reg, buf, 2);
  // Little-endian: LSB first, MSB second
  return static_cast<int16_t>(buf[0] | (buf[1] << 8));
}

std::array<int16_t, 3> BNO055I2C::readVector3(uint8_t start_reg)
{
  uint8_t buf[6] = {};
  readRegisters(start_reg, buf, 6);
  return {
    static_cast<int16_t>(buf[0] | (buf[1] << 8)),
    static_cast<int16_t>(buf[2] | (buf[3] << 8)),
    static_cast<int16_t>(buf[4] | (buf[5] << 8))
  };
}

std::array<int16_t, 4> BNO055I2C::readVector4(uint8_t start_reg)
{
  uint8_t buf[8] = {};
  readRegisters(start_reg, buf, 8);
  return {
    static_cast<int16_t>(buf[0] | (buf[1] << 8)),
    static_cast<int16_t>(buf[2] | (buf[3] << 8)),
    static_cast<int16_t>(buf[4] | (buf[5] << 8)),
    static_cast<int16_t>(buf[6] | (buf[7] << 8))
  };
}

}  // namespace bno055_driver
