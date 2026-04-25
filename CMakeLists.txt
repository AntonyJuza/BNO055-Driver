cmake_minimum_required(VERSION 3.8)
project(bno055_driver)

# Default to C++17 (ROS2 Jazzy standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── Build the library (I2C HAL + ROS2 node) ──────────────────────────────────
add_library(bno055_lib
  src/bno055_i2c.cpp
  src/bno055_node.cpp
)

target_include_directories(bno055_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bno055_lib
  rclcpp
  sensor_msgs
  diagnostic_msgs
  std_msgs
)

# ── Build the executable ─────────────────────────────────────────────────────
add_executable(bno055_node src/main.cpp)
target_link_libraries(bno055_node bno055_lib)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS bno055_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bno055_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ── Export ────────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(bno055_lib)
ament_export_dependencies(rclcpp sensor_msgs diagnostic_msgs std_msgs)

ament_package()
