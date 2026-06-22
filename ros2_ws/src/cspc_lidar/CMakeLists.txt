cmake_minimum_required(VERSION 3.8)
project(cspc_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cspc_lidar
  sdk/mtime.cpp
  sdk/node_lidar.cpp
  sdk/serial_port.cpp
  sdk/timer.cpp
  sdk/lidar_data_processing.cpp
  sdk/point_cloud_optimize.cpp
  sdk/calibration.cpp
  src/node_lidar_ros.cpp
)

target_include_directories(cspc_lidar PRIVATE
  ${PROJECT_SOURCE_DIR}/include/cspc_lidar
  ${PROJECT_SOURCE_DIR}/sdk
)
ament_target_dependencies(cspc_lidar rclcpp sensor_msgs std_msgs)

install(TARGETS cspc_lidar DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params rviz DESTINATION share/${PROJECT_NAME})

ament_package()
