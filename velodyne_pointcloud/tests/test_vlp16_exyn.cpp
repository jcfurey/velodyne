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
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>

#include "velodyne_pointcloud/calibration.hpp"
#include "velodyne_pointcloud/datacontainerbase.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace
{

struct DecodedPoint
{
  float x;
  float y;
  float z;
  float distance;
  float time;
};

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
    float x, float y, float z, const uint16_t, const float distance, const float,
    const float time) override
  {
    times.push_back(time);
    points.push_back({x, y, z, distance, time});
  }

  void newLine() override
  {
    ++lines;
  }

  int lines{0};
  std::vector<float> times;
  std::vector<DecodedPoint> points;
};

// Every return in a block reports that block's raw distance.
velodyne_msgs::msg::VelodynePacket makePacket(
  const std::vector<uint16_t> & block_azimuths, uint8_t return_mode,
  const std::vector<uint16_t> & block_distances = std::vector<uint16_t>(12, 0))
{
  EXPECT_EQ(block_azimuths.size(), 12U);
  EXPECT_EQ(block_distances.size(), 12U);
  velodyne_msgs::msg::VelodynePacket packet;
  packet.data.fill(0);
  for (size_t block_index = 0; block_index < block_azimuths.size(); ++block_index) {
    const size_t offset = block_index * velodyne_rawdata::SIZE_BLOCK;
    const uint16_t header = velodyne_rawdata::UPPER_BANK;
    std::memcpy(packet.data.data() + offset, &header, sizeof(header));
    std::memcpy(
      packet.data.data() + offset + sizeof(header),
      &block_azimuths[block_index], sizeof(block_azimuths[block_index]));
    for (int scan = 0; scan < velodyne_rawdata::SCANS_PER_BLOCK; ++scan) {
      std::memcpy(
        packet.data.data() + offset + 4 + scan * velodyne_rawdata::RAW_SCAN_SIZE,
        &block_distances[block_index], sizeof(block_distances[block_index]));
    }
  }
  packet.data[1204] = return_mode;
  packet.data[1205] = 0x22;
  packet.stamp.sec = 1;
  return packet;
}

velodyne_msgs::msg::VelodynePacket makeDualPacket(
  const std::vector<uint16_t> & logical_azimuths,
  const std::vector<uint16_t> & block_distances = std::vector<uint16_t>(12, 0))
{
  EXPECT_EQ(logical_azimuths.size(), 6U);
  std::vector<uint16_t> block_azimuths;
  for (const uint16_t azimuth : logical_azimuths) {
    block_azimuths.push_back(azimuth);
    block_azimuths.push_back(azimuth);
  }
  return makePacket(block_azimuths, 57, block_distances);
}

std::filesystem::path calibrationPath()
{
  std::filesystem::path package_prefix;
  ament_index_cpp::get_package_prefix("velodyne_pointcloud", package_prefix);
  return package_prefix / "share/velodyne_pointcloud/params/VLP16db.yaml";
}

std::unique_ptr<velodyne_rawdata::RawData> makeDefaultDecoder()
{
  auto decoder = std::make_unique<velodyne_rawdata::RawData>(
    calibrationPath().string(), "VLP16");
  decoder->setParameters(0.1, 100.0, 0.0, 2.0 * M_PI);
  return decoder;
}

std::unique_ptr<velodyne_rawdata::RawData> makeDecoder(
  const std::string & dual_return_mode = "last")
{
  auto decoder = makeDefaultDecoder();
  decoder->setVlp16DualReturnMode(dual_return_mode);
  decoder->setVlp16ScanBoundaryClipping(true);
  decoder->setVlp16PacketTimestampReference(4);
  return decoder;
}

// VLP16db.yaml has no rotational or horizontal offsets, so a point's XY
// bearing is the corrected azimuth that indexed the sine/cosine table.
int decodedAzimuth(const DecodedPoint & point)
{
  const double hundredths = std::atan2(-point.y, point.x) * 18000.0 / M_PI;
  return static_cast<int>(std::lround(hundredths + 36000.0)) % 36000;
}

int expectedAzimuth(float azimuth, float azimuth_diff, int firing, int dsr)
{
  const float corrected = azimuth + (azimuth_diff * (
      (dsr * velodyne_rawdata::VLP16_DSR_TOFFSET) +
      (firing * velodyne_rawdata::VLP16_FIRING_TOFFSET)) /
    velodyne_rawdata::VLP16_BLOCK_TDURATION);
  return static_cast<int>(std::round(corrected)) % 36000;
}

std::filesystem::path writeEccentricCalibration(const std::string & model)
{
  std::ifstream source(calibrationPath());
  std::stringstream contents;
  contents << source.rdbuf();
  const auto path = std::filesystem::temp_directory_path() /
    ("test_vlp16_exyn_" + std::to_string(::getpid()) + ".yaml");
  std::ofstream output(path);
  output << contents.str() << "\nencoder_eccentricity_model: [" << model << "]\n";
  return path;
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

TEST(Vlp16Exyn, dual_return_modes_follow_the_manual_block_order)
{
  // VLP-16 manual: the first block of each pair holds the last return and
  // the second holds the strongest return.
  std::vector<uint16_t> distances;
  for (int pair = 0; pair < 6; ++pair) {
    distances.push_back(6000);
    distances.push_back(2500);
  }
  const auto packet = makeDualPacket({100, 140, 180, 220, 260, 300}, distances);
  const auto decode = [&packet](const std::string & mode) {
      CountingContainer container;
      makeDecoder(mode)->unpack(
        packet, container, rclcpp::Time(int64_t{0}, RCL_ROS_TIME), false, false);
      return container.points;
    };

  const auto last = decode("last");
  const auto strongest = decode("strongest");
  ASSERT_EQ(last.size(), 192U);
  ASSERT_EQ(strongest.size(), 192U);
  for (size_t index = 0; index < last.size(); ++index) {
    EXPECT_NEAR(last[index].distance, 12.0F, 1e-4);
    EXPECT_NEAR(strongest[index].distance, 5.0F, 1e-4);
    EXPECT_FLOAT_EQ(last[index].time, strongest[index].time);
  }
}

TEST(Vlp16Exyn, clipped_first_packet_interpolates_a_wrap_in_the_last_block)
{
  std::vector<uint16_t> azimuths;
  for (uint16_t block = 0; block < 11; ++block) {
    azimuths.push_back(35577 + 40 * block);
  }
  azimuths.push_back(17);
  const auto packet = makePacket(azimuths, 55, std::vector<uint16_t>(12, 5000));
  CountingContainer container;

  makeDecoder()->unpack(
    packet, container, rclcpp::Time(int64_t{0}, RCL_ROS_TIME), true, false);

  ASSERT_EQ(container.points.size(), 32U);
  for (int firing = 0; firing < 2; ++firing) {
    for (int dsr = 0; dsr < 16; ++dsr) {
      EXPECT_EQ(
        decodedAzimuth(container.points[firing * 16 + dsr]),
        expectedAzimuth(17, 40.0F, firing, dsr));
    }
  }
}

TEST(Vlp16Exyn, clipped_first_packet_interpolates_a_wrap_in_the_last_pair)
{
  const auto packet = makeDualPacket(
    {35817, 35857, 35897, 35937, 35977, 17}, std::vector<uint16_t>(12, 5000));
  for (const std::string mode : {"last", "strongest"}) {
    CountingContainer container;

    makeDecoder(mode)->unpack(
      packet, container, rclcpp::Time(int64_t{0}, RCL_ROS_TIME), true, false);

    ASSERT_EQ(container.points.size(), 32U) << mode;
    for (int firing = 0; firing < 2; ++firing) {
      for (int dsr = 0; dsr < 16; ++dsr) {
        EXPECT_EQ(
          decodedAzimuth(container.points[firing * 16 + dsr]),
          expectedAzimuth(17, 40.0F, firing, dsr)) << mode;
      }
    }
  }
}

// With the Exyn options left at their defaults, a single-return packet keeps
// the upstream block timing and next-block azimuth interpolation.
TEST(Vlp16ExynDefaults, single_return_packet_matches_upstream)
{
  std::vector<uint16_t> azimuths;
  for (uint16_t block = 0; block < 12; ++block) {
    azimuths.push_back(1000 + 40 * block);
  }
  const auto packet = makePacket(azimuths, 55, std::vector<uint16_t>(12, 5000));
  CountingContainer container;

  makeDefaultDecoder()->unpack(
    packet, container, rclcpp::Time(int64_t{1000000000}, RCL_ROS_TIME));

  ASSERT_EQ(container.lines, 24);
  ASSERT_EQ(container.points.size(), 384U);
  for (int block = 0; block < 12; ++block) {
    for (int firing = 0; firing < 2; ++firing) {
      for (int dsr = 0; dsr < 16; ++dsr) {
        const auto & point = container.points[(block * 2 + firing) * 16 + dsr];
        EXPECT_NEAR(point.time, (110.592 * block + 55.296 * firing + 2.304 * dsr) * 1e-6, 1e-9);
        EXPECT_EQ(decodedAzimuth(point), expectedAzimuth(azimuths[block], 40.0F, firing, dsr));
      }
    }
  }
}

// Deliberate default-mode deviations for dual returns: upstream times dual
// block b as single-return block b and interpolates each first block against
// its same-azimuth partner. Here both blocks of pair p take pair p's timing
// and interpolate towards the next pair.
TEST(Vlp16ExynDefaults, dual_return_pairs_share_timing_and_interpolation)
{
  const std::vector<uint16_t> logical = {1000, 1040, 1080, 1120, 1160, 1200};
  const auto packet = makeDualPacket(logical, std::vector<uint16_t>(12, 5000));
  CountingContainer container;

  makeDefaultDecoder()->unpack(
    packet, container, rclcpp::Time(int64_t{1000000000}, RCL_ROS_TIME));

  ASSERT_EQ(container.lines, 24);
  ASSERT_EQ(container.points.size(), 384U);
  for (int block = 0; block < 12; ++block) {
    const int pair = block / 2;
    for (int firing = 0; firing < 2; ++firing) {
      for (int dsr = 0; dsr < 16; ++dsr) {
        const auto & point = container.points[(block * 2 + firing) * 16 + dsr];
        EXPECT_NEAR(point.time, (110.592 * pair + 55.296 * firing + 2.304 * dsr) * 1e-6, 1e-9);
        EXPECT_EQ(decodedAzimuth(point), expectedAzimuth(logical[pair], 40.0F, firing, dsr));
      }
    }
  }
}

// Deliberate default-mode deviation: upstream reuses the previous delta at
// an in-packet wrap and drops block 0 when no previous delta exists. Here
// the wrap uses the positive modulo delta and block 0 is kept.
TEST(Vlp16ExynDefaults, internal_wrap_uses_the_modulo_delta)
{
  std::vector<uint16_t> azimuths = {35980};
  for (uint16_t block = 1; block < 12; ++block) {
    azimuths.push_back(20 + 40 * (block - 1));
  }
  const auto packet = makePacket(azimuths, 55, std::vector<uint16_t>(12, 5000));
  CountingContainer container;

  makeDefaultDecoder()->unpack(
    packet, container, rclcpp::Time(int64_t{1000000000}, RCL_ROS_TIME));

  ASSERT_EQ(container.points.size(), 384U);
  for (int firing = 0; firing < 2; ++firing) {
    for (int dsr = 0; dsr < 16; ++dsr) {
      EXPECT_EQ(
        decodedAzimuth(container.points[firing * 16 + dsr]),
        expectedAzimuth(35980, 40.0F, firing, dsr));
    }
  }
}

TEST(Vlp16Exyn, eccentricity_model_is_bounded_and_inverted)
{
  const auto accepted_path = writeEccentricCalibration("0.0022, -0.0011");
  velodyne_pointcloud::Calibration calibration(accepted_path.string());
  std::filesystem::remove(accepted_path);
  ASSERT_TRUE(calibration.has_encoder_eccentricity_model);
  for (const double true_angle : {0.1, 1.7, 3.0, 4.6, 6.2}) {
    const float measured = static_cast<float>(
      std::atan2(std::sin(true_angle) - 0.0011, std::cos(true_angle) + 0.0022));
    const double error = calibration.correctedAzimuth(measured) - true_angle;
    EXPECT_NEAR(std::atan2(std::sin(error), std::cos(error)), 0.0, 2e-6);
  }

  // The inverse checks its final residual rather than returning a guess.
  calibration.encoder_eccentricity_model = {0.95, 0.0};
  EXPECT_THROW(calibration.correctedAzimuth(2.0F), std::runtime_error);

  const auto rejected_path = writeEccentricCalibration("0.2, 0.0");
  EXPECT_THROW(
    {velodyne_pointcloud::Calibration rejected(rejected_path.string());},
    std::runtime_error);
  std::filesystem::remove(rejected_path);
}

}  // namespace
