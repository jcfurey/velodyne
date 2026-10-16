// Copyright 2026 John Cameron Furey
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "velodyne_laserscan/velodyne_laserscan.hpp"

namespace
{
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using velodyne_laserscan::pointCloudToLaserScan;

PointCloud2 cloudWithPoints(uint32_t width, uint32_t height = 1, uint32_t padding = 0)
{
  PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.width = width;
  cloud.height = height;
  cloud.point_step = 22;
  cloud.row_step = width * cloud.point_step + padding;
  cloud.data.resize(static_cast<size_t>(height) * cloud.row_step, 0xff);
  for (const auto & definition : {std::make_pair("x", 0U), {"y", 4U}, {"intensity", 12U}}) {
    PointField field;
    field.name = definition.first;
    field.offset = definition.second;
    field.count = 1;
    field.datatype = PointField::FLOAT32;
    cloud.fields.push_back(field);
  }
  PointField ring;
  ring.name = "ring";
  ring.offset = 16;
  ring.count = 1;
  ring.datatype = PointField::UINT16;
  cloud.fields.push_back(ring);
  return cloud;
}

void setPoint(
  PointCloud2 & cloud, uint32_t row, uint32_t column,
  float x, float y, float intensity = 1.0f, uint16_t ring = 8)
{
  auto * point = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
  std::memcpy(point, &x, sizeof(x));
  std::memcpy(point + 4, &y, sizeof(y));
  std::memcpy(point + 12, &intensity, sizeof(intensity));
  std::memcpy(point + 16, &ring, sizeof(ring));
}

TEST(Projection, keeps_nearest_return_and_its_intensity_independent_of_order)
{
  auto cloud = cloudWithPoints(2);
  for (const bool reverse : {false, true}) {
    setPoint(cloud, 0, reverse ? 1 : 0, 2.0f, 0.0f, 25.0f);
    setPoint(cloud, 0, reverse ? 0 : 1, 7.0f, 0.0f, 75.0f);
    const auto scan = pointCloudToLaserScan(cloud, 8, 0.007);
    const auto bin = static_cast<size_t>(M_PI / scan.angle_increment);
    EXPECT_FLOAT_EQ(scan.ranges[bin], 2.0f);
    EXPECT_FLOAT_EQ(scan.intensities[bin], 25.0f);
    EXPECT_EQ(scan.header.frame_id, "lidar");
  }
}

TEST(Projection, handles_packed_points_row_padding_and_missing_intensity)
{
  auto cloud = cloudWithPoints(2, 2, 7);
  cloud.fields.erase(cloud.fields.begin() + 2);
  setPoint(cloud, 0, 0, 5.0f, 0.0f);
  setPoint(cloud, 0, 1, 4.0f, 0.0f);
  setPoint(cloud, 1, 0, 3.0f, 0.0f);
  setPoint(cloud, 1, 1, 2.0f, 0.0f);
  const auto scan = pointCloudToLaserScan(cloud, 8, 0.007);
  EXPECT_TRUE(scan.intensities.empty());
  EXPECT_FLOAT_EQ(*std::min_element(scan.ranges.begin(), scan.ranges.end()), 2.0f);
}

TEST(Projection, handles_big_endian_fields)
{
  auto cloud = cloudWithPoints(2);
  setPoint(cloud, 0, 0, 2.0f, 0.0f, 42.0f);
  setPoint(cloud, 0, 1, 3.0f, 0.0f, 43.0f);
  for (uint32_t column = 0; column < cloud.width; ++column) {
    for (const auto & field : cloud.fields) {
      auto begin = cloud.data.begin() + column * cloud.point_step + field.offset;
      std::reverse(begin, begin + (field.datatype == PointField::UINT16 ? 2 : 4));
    }
  }
  cloud.is_bigendian = true;
  const auto scan = pointCloudToLaserScan(cloud, 8, 0.007);
  EXPECT_FLOAT_EQ(*std::min_element(scan.ranges.begin(), scan.ranges.end()), 2.0f);
}

TEST(Projection, ignores_nonfinite_zero_and_out_of_range_returns)
{
  auto cloud = cloudWithPoints(4);
  setPoint(cloud, 0, 0, std::numeric_limits<float>::quiet_NaN(), 1.0f);
  setPoint(cloud, 0, 1, std::numeric_limits<float>::infinity(), 0.0f);
  setPoint(cloud, 0, 2, 0.0f, 0.0f);
  setPoint(cloud, 0, 3, 201.0f, 0.0f);
  const auto scan = pointCloudToLaserScan(cloud, 8, 0.007);
  EXPECT_TRUE(std::all_of(scan.ranges.begin(), scan.ranges.end(),
    [](float range) {return std::isinf(range);}));
}

TEST(Projection, wraps_positive_pi_and_publishes_consistent_angle_limits)
{
  auto cloud = cloudWithPoints(1);
  setPoint(cloud, 0, 0, -3.0f, 0.0f);
  const auto scan = pointCloudToLaserScan(cloud, 8, 0.007);
  EXPECT_FLOAT_EQ(scan.ranges.front(), 3.0f);
  EXPECT_NEAR(scan.angle_max,
    scan.angle_min + (scan.ranges.size() - 1) * scan.angle_increment, 1e-6);
}

TEST(Projection, accepts_an_empty_cloud)
{
  const auto scan = pointCloudToLaserScan(cloudWithPoints(0), -1, 0.007);
  EXPECT_FALSE(scan.ranges.empty());
}

TEST(Projection, rejects_truncated_or_invalid_layouts)
{
  auto cloud = cloudWithPoints(1);
  cloud.data.pop_back();
  EXPECT_THROW(pointCloudToLaserScan(cloud, 8, 0.007), std::invalid_argument);
  cloud = cloudWithPoints(1);
  cloud.fields[0].offset = 21;
  EXPECT_THROW(pointCloudToLaserScan(cloud, 8, 0.007), std::invalid_argument);
  cloud = cloudWithPoints(1);
  cloud.fields.back().datatype = PointField::FLOAT32;
  EXPECT_THROW(pointCloudToLaserScan(cloud, 8, 0.007), std::invalid_argument);
  cloud = cloudWithPoints(1);
  cloud.row_step = 1;
  EXPECT_THROW(pointCloudToLaserScan(cloud, 8, 0.007), std::invalid_argument);
}

TEST(Projection, validates_resolution_and_supports_all_128_rings)
{
  auto cloud = cloudWithPoints(1);
  setPoint(cloud, 0, 0, 2.0f, 0.0f, 1.0f, 127);
  const auto scan = pointCloudToLaserScan(cloud, 127, 0.007);
  EXPECT_FLOAT_EQ(*std::min_element(scan.ranges.begin(), scan.ranges.end()), 2.0f);
  EXPECT_THROW(pointCloudToLaserScan(cloud, 8, 0.0), std::invalid_argument);
  EXPECT_THROW(pointCloudToLaserScan(cloud, 8,
    std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}
}  // namespace
