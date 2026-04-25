# Copyright (c) 2026 Juza. MIT License.
# BNO055 ROS2 Driver — Launch file
#
# Standalone launch:
#   ros2 launch bno055_driver bno055.launch.py
#
# Override parameters:
#   ros2 launch bno055_driver bno055.launch.py i2c_bus:=/dev/i2c-3 update_rate:=30.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('bno055_driver')
    params_file = os.path.join(pkg, 'config', 'bno055_params.yaml')

    return LaunchDescription([

        # ── Launch arguments for override ─────────────────────────────────
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to BNO055 parameter YAML file'
        ),

        # ── BNO055 driver node ────────────────────────────────────────────
        Node(
            package='bno055_driver',
            executable='bno055_node',
            name='bno055_node',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            emulate_tty=True,
        ),
    ])
