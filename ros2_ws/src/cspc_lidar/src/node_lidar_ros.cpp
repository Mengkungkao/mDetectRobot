#include "node_lidar_ros.h"
#include "node_lidar.h"
#include "lidar_information.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

using namespace std::chrono_literals;

static sensor_msgs::msg::PointCloud2 scan_to_cloud(
    const LaserScan &scan,
    const std::string &frame_id,
    const rclcpp::Time &stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(scan.points.size());
  cloud.is_dense = false;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(scan.points.size());

  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  for (const auto &point : scan.points) {
    const float angle_rad = point.angle * static_cast<float>(M_PI / 180.0);
    *out_x = point.range * std::cos(angle_rad);
    *out_y = point.range * std::sin(angle_rad);
    *out_z = 0.0F;
    ++out_x;
    ++out_y;
    ++out_z;
  }
  return cloud;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cspc_lidar");

  node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  node->declare_parameter<int>("baudrate", 230400);
  node->declare_parameter<std::string>("frame_id", "laser");
  node->declare_parameter<int>("version", 4);

  node_lidar.lidar_general_info.port = node->get_parameter("port").as_string();
  node_lidar.lidar_general_info.m_SerialBaudrate = node->get_parameter("baudrate").as_int();
  node_lidar.lidar_general_info.frame_id = node->get_parameter("frame_id").as_string();
  node_lidar.lidar_general_info.version = node->get_parameter("version").as_int();

  auto error_pub = node->create_publisher<std_msgs::msg::String>("lsd_error", 10);
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS());

  auto status_sub = node->create_subscription<std_msgs::msg::UInt16>(
      "lidar_status", 10,
      [](const std_msgs::msg::UInt16::SharedPtr msg) {
        if (!node_lidar.serial_port) {
          return;
        }
        switch (msg->data) {
          case 1:
            node_lidar.lidar_status.lidar_ready = true;
            node_lidar.lidar_status.lidar_abnormal_state = 0;
            break;
          case 2:
            node_lidar.lidar_status.lidar_ready = false;
            node_lidar.lidar_status.close_lidar = true;
            node_lidar.serial_port->write_data(end_lidar, 4);
            break;
          case 3:
            node_lidar.serial_port->write_data(high_exposure, 4);
            break;
          case 4:
            node_lidar.serial_port->write_data(low_exposure, 4);
            break;
          case 5:
            node_lidar.lidar_status.lidar_abnormal_state = 0;
            break;
          case 6:
            node_lidar.serial_port->write_data(high_speed, 4);
            break;
          case 7:
            node_lidar.serial_port->write_data(low_speed, 4);
            break;
          default:
            break;
        }
      });
  (void)status_sub;

  RCLCPP_INFO(node->get_logger(), "Opening COIN-D6 LiDAR on %s at %d baud (SDK version %d)",
              node_lidar.lidar_general_info.port.c_str(),
              node_lidar.lidar_general_info.m_SerialBaudrate,
              node_lidar.lidar_general_info.version);

  if (node_start() != 0) {
    RCLCPP_FATAL(node->get_logger(), "COIN-D6 SDK failed to initialize the serial device");
    rclcpp::shutdown();
    return 1;
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (node_lidar.lidar_status.lidar_abnormal_state != 0) {
      std_msgs::msg::String message;
      message.data = "COIN-D6 abnormal state: " +
          std::to_string(node_lidar.lidar_status.lidar_abnormal_state);
      error_pub->publish(message);
    }

    LaserScan scan;
    if (!data_handling(scan)) {
      std::this_thread::sleep_for(5ms);
      continue;
    }

    const auto stamp = node->now();
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = stamp;
    scan_msg.header.frame_id = node_lidar.lidar_general_info.frame_id;
    scan_msg.angle_min = scan.config.min_angle;
    scan_msg.angle_max = scan.config.max_angle;
    scan_msg.angle_increment = scan.config.angle_increment;
    scan_msg.scan_time = scan.config.scan_time;
    scan_msg.time_increment = scan.config.time_increment;
    scan_msg.range_min = scan.config.min_range;
    scan_msg.range_max = scan.config.max_range;
    scan_msg.ranges.reserve(scan.points.size());
    scan_msg.intensities.reserve(scan.points.size());
    for (const auto &point : scan.points) {
      scan_msg.ranges.push_back(point.range > 0.0F ? point.range : std::numeric_limits<float>::infinity());
      scan_msg.intensities.push_back(static_cast<float>(point.intensity));
    }

    laser_pub->publish(scan_msg);
    cloud_pub->publish(scan_to_cloud(scan, node_lidar.lidar_general_info.frame_id, stamp));
  }

  if (node_lidar.serial_port) {
    node_lidar.serial_port->write_data(end_lidar, 4);
  }
  rclcpp::shutdown();
  return 0;
}
