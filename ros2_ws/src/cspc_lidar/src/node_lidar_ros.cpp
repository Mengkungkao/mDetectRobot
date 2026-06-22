#include "node_lidar.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}
}  // namespace

class CspcLidarNode : public rclcpp::Node
{
public:
  CspcLidarNode()
  : Node("cspc_lidar")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 230400);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    version_ = declare_parameter<int>("version", 4);
    range_min_ = declare_parameter<double>("range_min", 0.10);
    range_max_ = declare_parameter<double>("range_max", 10.0);
    angle_bins_ = std::max(90, declare_parameter<int>("angle_bins", 720));
    reverse_scan_ = declare_parameter<bool>("reverse_scan", true);
    angle_offset_rad_ = declare_parameter<double>("angle_offset_deg", 0.0) * kPi / 180.0;
    publish_point_cloud_ = declare_parameter<bool>("publish_point_cloud", false);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    error_pub_ = create_publisher<std_msgs::msg::String>("lsd_error", 10);
    if (publish_point_cloud_) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS());
    }

    status_sub_ = create_subscription<std_msgs::msg::UInt16>(
      "lidar_status", 10,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) { handle_status(msg->data); });

    node_lidar.lidar_general_info.port = port_;
    node_lidar.lidar_general_info.m_SerialBaudrate = baudrate_;
    node_lidar.lidar_general_info.frame_id = frame_id_;
    node_lidar.lidar_general_info.version = version_;

    RCLCPP_INFO(
      get_logger(),
      "Starting COIN-D6 SDK on %s @ %d baud, frame=%s, version=%d",
      port_.c_str(), baudrate_, frame_id_.c_str(), version_);

    if (node_start() != 0) {
      throw std::runtime_error("COIN-D6 SDK failed to open the serial device");
    }
  }

  ~CspcLidarNode() override
  {
    if (node_lidar.serial_port) {
      node_lidar.serial_port->write_data(end_lidar, 4);
    }
  }

  void poll_once()
  {
    LaserScan raw_scan;
    if (!data_handling(raw_scan)) {
      publish_sdk_error_if_needed();
      return;
    }

    auto msg = make_scan(raw_scan);
    scan_pub_->publish(msg);
    if (publish_point_cloud_ && cloud_pub_) {
      cloud_pub_->publish(make_cloud(msg));
    }
    publish_sdk_error_if_needed();
  }

private:
  sensor_msgs::msg::LaserScan make_scan(const LaserScan & raw_scan)
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.angle_min = -kPi;
    msg.angle_max = kPi;
    msg.angle_increment = (2.0 * kPi) / static_cast<double>(angle_bins_ - 1);
    msg.range_min = static_cast<float>(range_min_);
    msg.range_max = static_cast<float>(range_max_);
    msg.scan_time = raw_scan.config.scan_time > 0.0F ? raw_scan.config.scan_time : 0.1F;
    msg.time_increment = msg.scan_time / static_cast<float>(angle_bins_);
    msg.ranges.assign(angle_bins_, std::numeric_limits<float>::infinity());
    msg.intensities.assign(angle_bins_, 0.0F);

    for (const auto & point : raw_scan.points) {
      if (!std::isfinite(point.range) || point.range < range_min_ || point.range > range_max_) {
        continue;
      }

      double angle = static_cast<double>(point.angle) * kPi / 180.0;
      if (reverse_scan_) angle = -angle;
      angle = normalize_angle(angle + angle_offset_rad_);
      const int index = static_cast<int>(
        std::llround((angle - msg.angle_min) / msg.angle_increment));
      if (index < 0 || index >= angle_bins_) continue;

      if (!std::isfinite(msg.ranges[index]) || point.range < msg.ranges[index]) {
        msg.ranges[index] = point.range;
        msg.intensities[index] = static_cast<float>(point.intensity);
      }
    }
    return msg;
  }

  sensor_msgs::msg::PointCloud2 make_cloud(const sensor_msgs::msg::LaserScan & scan)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    std::size_t valid = 0;
    for (const float range : scan.ranges) {
      if (std::isfinite(range)) ++valid;
    }
    modifier.resize(valid);

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range)) continue;
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      *x = range * std::cos(angle);
      *y = range * std::sin(angle);
      *z = 0.0F;
      ++x; ++y; ++z;
    }
    return cloud;
  }

  void handle_status(uint16_t command)
  {
    if (!node_lidar.serial_port) return;
    switch (command) {
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
        RCLCPP_WARN(get_logger(), "Unknown lidar_status command: %u", command);
        break;
    }
  }

  void publish_sdk_error_if_needed()
  {
    const uint8_t state = node_lidar.lidar_status.lidar_abnormal_state;
    if (state == 0 || state == last_error_state_) return;
    last_error_state_ = state;
    std_msgs::msg::String message;
    message.data = "COIN-D6 SDK abnormal state: " + std::to_string(state);
    error_pub_->publish(message);
    RCLCPP_ERROR(get_logger(), "%s", message.data.c_str());
  }

  std::string port_;
  std::string frame_id_;
  int baudrate_;
  int version_;
  int angle_bins_;
  double range_min_;
  double range_max_;
  double angle_offset_rad_;
  bool reverse_scan_;
  bool publish_point_cloud_;
  uint8_t last_error_state_{0};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr status_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CspcLidarNode>();
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      node->poll_once();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cspc_lidar"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
