// Copyright 2026 Exyn archive maintainers
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
// * Neither the name of the copyright holder nor the names of its
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

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <vector>

#include "velodyne_pointcloud/datacontainerbase.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace
{

class CountingContainer final : public velodyne_rawdata::DataContainerBase
{
public:
  CountingContainer()
  : DataContainerBase(
      0.0, 100.0, "", "", 16, 0, false, 384,
      std::make_shared<rclcpp::Clock>(), 1, "x", 1,
      sensor_msgs::msg::PointField::FLOAT32)
  {
  }

  void addPoint(
    float, float, float, const uint16_t, const float, const float,
    const float time) override
  {
    times.push_back(time);
  }

  void newLine() override
  {
    ++lines;
  }

  int lines{0};
  std::vector<float> times;
};

velodyne_msgs::msg::VelodynePacket makeDualPacket(
  const std::vector<uint16_t> & logical_azimuths)
{
  EXPECT_EQ(logical_azimuths.size(), 6U);
  velodyne_msgs::msg::VelodynePacket packet;
  packet.data.fill(0);
  for (size_t logical = 0; logical < logical_azimuths.size(); ++logical) {
    for (size_t return_index = 0; return_index < 2; ++return_index) {
      const size_t block_index = logical * 2 + return_index;
      const size_t offset = block_index * velodyne_rawdata::SIZE_BLOCK;
      const uint16_t header = velodyne_rawdata::UPPER_BANK;
      std::memcpy(packet.data.data() + offset, &header, sizeof(header));
      std::memcpy(
        packet.data.data() + offset + sizeof(header),
        &logical_azimuths[logical], sizeof(logical_azimuths[logical]));
    }
  }
  packet.data[1204] = 57;
  packet.stamp.sec = 1;
  return packet;
}

std::unique_ptr<velodyne_rawdata::RawData> makeDecoder()
{
  std::filesystem::path package_prefix;
  ament_index_cpp::get_package_prefix("velodyne_pointcloud", package_prefix);
  auto decoder = std::make_unique<velodyne_rawdata::RawData>(
    (package_prefix / "share/velodyne_pointcloud/params/VLP16db.yaml").string(),
    "VLP16");
  decoder->setParameters(0.1, 100.0, 0.0, 2.0 * M_PI);
  decoder->setVlp16DualReturnMode("strongest");
  decoder->setVlp16ScanBoundaryClipping(true);
  decoder->setVlp16PacketTimestampReference(4);
  return decoder;
}

TEST(Vlp16Exyn, clips_first_packet_before_internal_wrap)
{
  auto decoder = makeDecoder();
  auto packet = makeDualPacket({35858, 35897, 35937, 35977, 17, 57});
  CountingContainer container;

  decoder->unpack(
    packet, container, rclcpp::Time(int64_t{0}, RCL_ROS_TIME), true, false);

  EXPECT_EQ(container.lines, 4);
  EXPECT_EQ(container.times.size(), 64U);
}

TEST(Vlp16Exyn, clips_last_packet_after_internal_wrap_and_references_time)
{
  auto decoder = makeDecoder();
  auto packet = makeDualPacket({35984, 26, 65, 105, 144, 185});
  CountingContainer container;
  const rclcpp::Time boundary_time(int64_t{999668224}, RCL_ROS_TIME);

  decoder->unpack(packet, container, boundary_time, false, true);

  ASSERT_EQ(container.lines, 2);
  ASSERT_EQ(container.times.size(), 32U);
  EXPECT_NEAR(container.times.front(), -110.592e-6, 1e-9);
  EXPECT_NEAR(container.times.back(), -20.736e-6, 1e-9);
}

TEST(Vlp16Exyn, leaves_an_edge_packet_without_a_wrap_intact)
{
  auto decoder = makeDecoder();
  auto packet = makeDualPacket({100, 140, 180, 220, 260, 300});
  CountingContainer container;

  decoder->unpack(
    packet, container, rclcpp::Time(int64_t{0}, RCL_ROS_TIME), false, true);

  EXPECT_EQ(container.lines, 12);
  EXPECT_EQ(container.times.size(), 192U);
}

}  // namespace
