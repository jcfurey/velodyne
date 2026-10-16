// Copyright 2018, 2019 Kevin Hallenbeck, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "velodyne_laserscan/velodyne_laserscan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace velodyne_laserscan
{
namespace
{
template<typename T>
T readField(const uint8_t * source, bool swap_bytes)
{
  T value;
  if (swap_bytes) {
    uint8_t bytes[sizeof(T)];
    std::reverse_copy(source, source + sizeof(T), bytes);
    std::memcpy(&value, bytes, sizeof(value));
  } else {
    std::memcpy(&value, source, sizeof(value));
  }
  return value;
}

int fieldOffset(
  const sensor_msgs::msg::PointCloud2 & cloud, const char * name, uint8_t type, size_t size)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      if (field.datatype != type || field.count != 1 || field.offset > cloud.point_step ||
        size > cloud.point_step - field.offset ||
        field.offset > static_cast<uint32_t>(std::numeric_limits<int>::max()))
      {
        throw std::invalid_argument("Invalid PointCloud2 field layout");
      }
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}
}  // namespace

sensor_msgs::msg::LaserScan pointCloudToLaserScan(
  const sensor_msgs::msg::PointCloud2 & cloud, int selected_ring, double resolution)
{
  if (!std::isfinite(resolution) || resolution < 0.001 || resolution > 0.05 ||
    selected_ring < -1 || selected_ring > 127)
  {
    throw std::invalid_argument("Invalid ring or angular resolution");
  }
  const int offset_x = fieldOffset(cloud, "x", sensor_msgs::msg::PointField::FLOAT32, 4);
  const int offset_y = fieldOffset(cloud, "y", sensor_msgs::msg::PointField::FLOAT32, 4);
  const int offset_r = fieldOffset(cloud, "ring", sensor_msgs::msg::PointField::UINT16, 2);
  const int offset_i = fieldOffset(cloud, "intensity", sensor_msgs::msg::PointField::FLOAT32, 4);
  if (offset_x < 0 || offset_y < 0 || offset_r < 0 || cloud.point_step == 0 ||
    static_cast<uint64_t>(cloud.width) * cloud.point_step > cloud.row_step ||
    static_cast<uint64_t>(cloud.height) * cloud.row_step > cloud.data.size())
  {
    throw std::invalid_argument("PointCloud2 missing required fields or truncated data");
  }
  const uint16_t endian_probe = 1;
  const bool host_big_endian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
  const bool swap_bytes = cloud.is_bigendian != host_big_endian;
  if (selected_ring < 0) {
    uint32_t ring_count = 0;
    for (uint32_t row = 0; row < cloud.height; ++row) {
      const auto * point = cloud.data.data() + static_cast<size_t>(row) * cloud.row_step;
      for (uint32_t column = 0; column < cloud.width; ++column, point += cloud.point_step) {
        ring_count = std::max(
          ring_count, static_cast<uint32_t>(readField<uint16_t>(point + offset_r, swap_bytes)) + 1);
      }
    }
    selected_ring = ring_count > 64 ? 71 : ring_count > 32 ? 57 : ring_count > 16 ? 23 : 8;
    if (ring_count && static_cast<uint32_t>(selected_ring) >= ring_count) {
      selected_ring = ring_count / 2;
    }
  }
  sensor_msgs::msg::LaserScan scan;
  scan.header = cloud.header;
  scan.angle_increment = static_cast<float>(resolution);
  scan.angle_min = -static_cast<float>(M_PI);
  // Use the published float increment for indexing and metadata alike.
  const size_t bins = static_cast<size_t>(std::ceil(2.0 * M_PI / scan.angle_increment));
  scan.angle_max = scan.angle_min + (bins - 1) * scan.angle_increment;
  scan.range_min = 0.0f;
  scan.range_max = 200.0f;
  scan.ranges.assign(bins, std::numeric_limits<float>::infinity());
  if (offset_i >= 0) {
    scan.intensities.resize(bins);
  }
  for (uint32_t row = 0; row < cloud.height; ++row) {
    const auto * point = cloud.data.data() + static_cast<size_t>(row) * cloud.row_step;
    for (uint32_t column = 0; column < cloud.width; ++column, point += cloud.point_step) {
      if (readField<uint16_t>(point + offset_r, swap_bytes) != selected_ring) {
        continue;
      }
      const float x = readField<float>(point + offset_x, swap_bytes);
      const float y = readField<float>(point + offset_y, swap_bytes);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      const float range = std::hypot(x, y);
      if (range <= scan.range_min || range > scan.range_max) {
        continue;
      }
      double angle = std::atan2(y, x) + M_PI;
      if (angle >= 2.0 * M_PI) {
        angle = 0.0;
      }
      const size_t bin = static_cast<size_t>(angle / scan.angle_increment);
      if (bin < bins && range < scan.ranges[bin]) {
        scan.ranges[bin] = range;
        if (offset_i >= 0) {
          scan.intensities[bin] = readField<float>(point + offset_i, swap_bytes);
        }
      }
    }
  }
  return scan;
}

VelodyneLaserScan::VelodyneLaserScan(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_laserscan_node", options)
{
  rcl_interfaces::msg::ParameterDescriptor ring_desc;
  ring_desc.read_only = true;
  ring_desc.description = "Laser ring, or -1 to select a default based on the cloud";
  rcl_interfaces::msg::IntegerRange ring_range;
  ring_range.from_value = -1;
  ring_range.to_value = 127;
  ring_desc.integer_range.push_back(ring_range);
  ring_ = declare_parameter("ring", -1, ring_desc);

  rcl_interfaces::msg::ParameterDescriptor resolution_desc;
  resolution_desc.read_only = true;
  resolution_desc.description = "Angular resolution in radians";
  rcl_interfaces::msg::FloatingPointRange resolution_range;
  resolution_range.from_value = 0.001;
  resolution_range.to_value = 0.05;
  resolution_desc.floating_point_range.push_back(resolution_range);
  resolution_ = declare_parameter("resolution", 0.007, resolution_desc);
  if (!std::isfinite(resolution_)) {
    throw std::invalid_argument("resolution must be finite");
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::SensorDataQoS(),
    std::bind(&VelodyneLaserScan::recvCallback, this, std::placeholders::_1), subscription_options);
  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(10), publisher_options);
}

void VelodyneLaserScan::recvCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (pub_->get_subscription_count() == 0 && pub_->get_intra_process_subscription_count() == 0) {
    return;
  }
  try {
    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>(
      pointCloudToLaserScan(*msg, ring_, resolution_));
    pub_->publish(std::move(scan));
  } catch (const std::invalid_argument & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
  }
}

}  // namespace velodyne_laserscan

RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_laserscan::VelodyneLaserScan)
