// Copyright 2019 Sebastian Pütz, Joshua Whitley
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

#include <cmath>
#include <memory>
#include <cstring>
#include <limits>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>

#include "velodyne_pointcloud/organized_cloudXYZIRT.hpp"

namespace velodyne_pointcloud
{

OrganizedCloudXYZIRT::OrganizedCloudXYZIRT(
  const double min_range, const double max_range,
  const std::string & target_frame, const std::string & fixed_frame,
  const unsigned int num_lasers, const unsigned int scans_per_block,
  rclcpp::Clock::SharedPtr clock)
: DataContainerBase(
    min_range, max_range, target_frame, fixed_frame,
    num_lasers, 0, false, num_lasers == 64 ? 2 * scans_per_block : scans_per_block, clock, 6,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "ring", 1, sensor_msgs::msg::PointField::UINT16,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32)
{
  if (num_lasers == 0 || num_lasers > 128) {
    throw std::invalid_argument("Invalid organized cloud laser count");
  }
}

void OrganizedCloudXYZIRT::newLine()
{
  if ((static_cast<size_t>(cloud.height) + 1) * cloud.row_step > cloud.data.size()) {
    throw std::length_error("Decoded rows exceed scan capacity");
  }
  ++cloud.height;
}

void OrganizedCloudXYZIRT::setup(const velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan_msg)
{
  DataContainerBase::setup(scan_msg);
  // View filtering can skip individual rings. All missing slots must be NaN,
  // including their time, rather than appearing as valid points at the origin.
  const float missing = std::numeric_limits<float>::quiet_NaN();
  for (size_t offset = 0; offset < cloud.data.size(); offset += cloud.point_step) {
    auto * point = cloud.data.data() + offset;
    const uint16_t ring = (offset / cloud.point_step) % cloud.width;
    std::memcpy(point, &missing, sizeof(missing));
    std::memcpy(point + 4, &missing, sizeof(missing));
    std::memcpy(point + 8, &missing, sizeof(missing));
    std::memcpy(point + 12, &missing, sizeof(missing));
    std::memcpy(point + 16, &ring, sizeof(ring));
    std::memcpy(point + 18, &missing, sizeof(missing));
  }
}

void OrganizedCloudXYZIRT::addPoint(
  float x, float y, float z, const uint16_t ring,
  const float distance, const float intensity, const float time)
{
  const size_t offset = (static_cast<size_t>(cloud.height) * cloud.width + ring) *
    cloud.point_step;
  if (ring >= cloud.width || offset + cloud.point_step > cloud.data.size()) {
    throw std::out_of_range("Decoded ring exceeds organized cloud capacity");
  }
  float output_intensity = intensity;
  if (pointInRange(distance)) {
    transformPoint(x, y, z);
  } else {
    x = y = z = output_intensity = std::numeric_limits<float>::quiet_NaN();
  }
  auto * point = cloud.data.data() + offset;
  std::memcpy(point, &x, sizeof(x));
  std::memcpy(point + 4, &y, sizeof(y));
  std::memcpy(point + 8, &z, sizeof(z));
  std::memcpy(point + 12, &output_intensity, sizeof(output_intensity));
  std::memcpy(point + 16, &ring, sizeof(ring));
  std::memcpy(point + 18, &time, sizeof(time));
}

}  // namespace velodyne_pointcloud
