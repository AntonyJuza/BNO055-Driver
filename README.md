# BNO055 ROS2 Jazzy Driver

A lightweight ROS2 Jazzy **C++ driver** for the [Bosch BNO055](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) 9-axis absolute orientation IMU over I2C.  
Built for **Raspberry Pi 4 Model B** and designed for **SLAM mapping** with `robot_localization` EKF and `slam_toolbox`.

---

## Features

- **On-chip sensor fusion** — uses BNO055's NDOF mode (ARM Cortex M0+ running Bosch fusion), providing drift-corrected quaternion orientation from accelerometer + gyroscope + magnetometer
- **ROS2 standard messages** — publishes `sensor_msgs/Imu` with properly populated covariance matrices
- **Calibration diagnostics** — reports sys/gyro/accel/mag calibration levels (0–3) at 1 Hz
- **Temperature monitoring** — chip temperature published on a dedicated topic
- **Configurable** — I2C bus, address, polling rate, operation mode, frame ID, and mounting axis remap all set via YAML parameters
- **Hardware Axis Remap** — supports all 8 standard mounting positions (P0-P7) directly at the hardware scale, solving upside-down or rotated mountings
- **Clean architecture** — hardware I2C layer fully separated from ROS2 logic
- **No external dependencies** — uses Linux `i2c-dev` kernel interface directly; no vendor SDKs needed

## Topics

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/imu/bno055` | `sensor_msgs/Imu` | 50 Hz | Fused orientation (quaternion) + angular velocity + linear acceleration |
| `/imu/bno055/status` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Calibration status for sys, gyro, accel, mag (each 0–3) |
| `/imu/bno055/temperature` | `sensor_msgs/Temperature` | 50 Hz | Chip temperature in °C |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `i2c_bus` | `/dev/i2c-1` | I2C device path |
| `i2c_address` | `0x28` | BNO055 I2C address (`0x28` or `0x29`) |
| `frame_id` | `imu_link` | TF frame ID (should match your URDF) |
| `update_rate` | `50.0` | Data polling rate in Hz |
| `operation_mode` | `NDOF` | Fusion mode: `NDOF`, `NDOF_FMC_OFF`, or `IMU` |
| `publish_diagnostics` | `true` | Enable calibration status topic |
| `publish_temperature` | `true` | Enable temperature topic |
| `placement_axis_remap`| `P1` | Hardware axis remap for mounting orientation (P0-P7) |

---

## Hardware Setup

### Wiring (RPi4 → BNO055)

| RPi4 Pin | BNO055 Pin |
|---|---|
| Pin 3 (GPIO 2 — SDA) | SDA |
| Pin 5 (GPIO 3 — SCL) | SCL |
| Pin 1 (3.3V) | VIN |
| Pin 9 (GND) | GND |

> **Note:** Leave the ADR pin unconnected for the default address `0x28`.

### Verify I2C Connection

```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

You should see `28` in the output grid:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
...
```

### I2C Clock Stretching (if needed)

The BNO055 uses I2C clock stretching. If you get communication errors, switch to software I2C by adding this to `/boot/firmware/config.txt`:

```
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=2,i2c_gpio_scl=3
```

Reboot and update the parameter to `i2c_bus: "/dev/i2c-3"`.

### Axis Remapping (Mounting Orientation)

If your BNO055 is not mounted completely flat with the chip facing up and the connector to the left, you'll need to configure the axis remap parameter so the fused data rotates properly at a hardware level.

| Position | Description |
|---|---|
| **P0** | Chip facing up, connector right (Default at POR) |
| **P1** | Chip facing up, connector left (**BNO055 default**) |
| **P2** | Chip facing up, connector down |
| **P3** | Chip facing up, connector up |
| **P4** | Chip facing DOWN (**upside down**) |
| **P5** | Rotated 90° CW |
| **P6** | Rotated 180° |
| **P7** | Rotated 270° CW |

In `bno055_params.yaml`, set `placement_axis_remap: "P4"` (for upside down, for example). This writes directly to the BNO055's `AXIS_MAP_CONFIG` and `AXIS_MAP_SIGN` registers, efficiently fixing axes before fusion.

---

## Installation

### Prerequisites

- ROS2 Jazzy on Raspberry Pi 4 (Ubuntu 24.04)
- I2C enabled (`sudo raspi-config` → Interface → I2C → Enable)
- User in the `i2c` group: `sudo usermod -aG i2c $USER`

### Build

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone <this-repo-url> bno055_driver

# Build
cd ~/ros2_ws
colcon build --packages-select bno055_driver
source install/setup.bash
```

---

## Usage

### Standalone Launch

```bash
ros2 launch bno055_driver bno055.launch.py
```

### With Custom Parameters

```bash
ros2 launch bno055_driver bno055.launch.py \
  params_file:=/path/to/your/bno055_params.yaml
```

### Direct Run (no launch file)

```bash
ros2 run bno055_driver bno055_node --ros-args \
  -p i2c_bus:=/dev/i2c-1 \
  -p i2c_address:=0x28 \
  -p frame_id:=imu_link \
  -p update_rate:=50.0
```

### Include in Another Launch File

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

bno055_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('bno055_driver'),
            'launch', 'bno055.launch.py'
        )
    ),
)
```

---

## Calibration

The BNO055 requires calibration after every power cycle. Monitor calibration via:

```bash
ros2 topic echo /imu/bno055/status
```

Each subsystem reports a level from **0** (uncalibrated) to **3** (fully calibrated):

| Sensor | How to Calibrate |
|---|---|
| **Gyroscope** | Hold the sensor still for 3–5 seconds |
| **Accelerometer** | Place on 6 different flat surfaces (often auto-calibrates) |
| **Magnetometer** | Move the sensor in a **figure-8 pattern** in the air |
| **System** | Reaches 3/3 automatically once all sub-sensors are calibrated |

> **Tip:** The accelerometer and gyroscope usually auto-calibrate within seconds. The magnetometer takes the most effort — keep doing figure-8s until `calib_mag` hits 3/3.

---

## Integration with robot_localization EKF

Add the BNO055 as an IMU source in your `ekf.yaml`:

```yaml
ekf_filter_node:
  ros__parameters:
    imu0: /imu/bno055
    imu0_config: [false, false, false,   # x, y, z
                  false, false, false,   # roll, pitch, yaw
                  false, false, false,   # vx, vy, vz
                  false, false, true,    # vroll, vpitch, vyaw
                  true,  false, false]   # ax, ay, az
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true
```

> **Note:** This driver publishes **linear acceleration** (gravity already removed by BNO055 fusion engine). Setting `imu0_remove_gravitational_acceleration: true` is still recommended as a safety net.

---

## Package Structure

```
bno055_driver/
├── CMakeLists.txt                          # ament_cmake build
├── package.xml                             # Package manifest
├── README.md
├── config/
│   └── bno055_params.yaml                  # Default parameters
├── launch/
│   └── bno055.launch.py                    # Standalone launch file
├── include/bno055_driver/
│   ├── bno055_i2c.hpp                      # I2C HAL — register map, driver class
│   └── bno055_node.hpp                     # ROS2 node header
└── src/
    ├── bno055_i2c.cpp                      # I2C implementation (Linux i2c-dev)
    ├── bno055_node.cpp                     # ROS2 node — polling, publishing
    └── main.cpp                            # Entry point
```

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  bno055_node                    │
│  (rclcpp::Node — timer-based polling @ 50 Hz)   │
│                                                 │
│  Publishers:                                    │
│    /imu/bno055          → sensor_msgs/Imu       │
│    /imu/bno055/status   → DiagnosticArray       │
│    /imu/bno055/temperature → Temperature        │
├─────────────────────────────────────────────────┤
│                  bno055_i2c                     │
│  (Pure C++ — no ROS2 dependency)                │
│                                                 │
│  I2C_RDWR ioctl → repeated-start transactions  │
│  Register reads: quaternion, gyro, accel, temp  │
│  Init: reset → verify chip ID → set NDOF mode  │
└──────────────────────┬──────────────────────────┘
                       │ /dev/i2c-1
                 ┌─────┴─────┐
                 │  BNO055   │
                 │  (0x28)   │
                 └───────────┘
```

---

## Operation Modes

| Mode | Sensors Used | Use Case |
|---|---|---|
| `NDOF` | Accel + Gyro + Mag | **Recommended for SLAM** — full absolute orientation |
| `NDOF_FMC_OFF` | Accel + Gyro + Mag | Same as NDOF without fast magnetometer calibration |
| `IMU` | Accel + Gyro only | Indoor use where magnetometer interference is severe |

---

## Troubleshooting

| Problem | Solution |
|---|---|
| `Failed to open I2C bus` | Check I2C is enabled: `ls /dev/i2c-*` |
| `Chip ID mismatch` | Verify wiring, check `i2cdetect -y 1` shows `0x28` |
| `I2C read/write failed` (frequent) | Enable software I2C — see clock stretching section above |
| Quaternion reads all zeros | BNO055 may need reset — power cycle the sensor |
| Calibration stuck at 0 | Magnetometer needs figure-8 motion; keep sensor away from metal |
| Permission denied on `/dev/i2c-1` | Run `sudo usermod -aG i2c $USER` and re-login |

---

## License

MIT License — see [package.xml](package.xml).

## Author

**Juza** — [Antonyjuza@gmail.com](mailto:Antonyjuza@gmail.com)
