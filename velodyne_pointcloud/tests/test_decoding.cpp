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
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include "test_utils.hpp"

#include "velodyne_pointcloud/calibration.hpp"
#include "velodyne_pointcloud/organized_cloudXYZIRT.hpp"
#include "velodyne_pointcloud/pointcloudXYZIRT.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace
{
std::string calibrationPath(const std::string & name)
{
  return (velodyne_test::packageSharePath() / "params" / name).string();
}

velodyne_msgs::msg::VelodynePacket packetFor(const std::string & model, uint16_t azimuth = 1000)
{
  velodyne_msgs::msg::VelodynePacket packet;
  packet.stamp.sec = 1;
  for (int block = 0; block < 12; ++block) {
    auto * bytes = packet.data.data() + block * 100;
    bytes[0] = 0xff;
    bytes[1] = model == "VLS128" ? static_cast<uint8_t>(0xee - 0x11 * (block % 4)) : 0xee;
    const uint16_t angle = azimuth + (model == "VLS128" ? block / 4 : block) * 40;
    bytes[2] = angle & 0xff;
    bytes[3] = angle >> 8;
    for (int point = 0; point < 32; ++point) {
      bytes[4 + point * 3] = 0xc4;
      bytes[5 + point * 3] = 0x09;  // raw distance 2500
      bytes[6 + point * 3] = 20;
    }
  }
  packet.data[1204] = 55;
  packet.data[1205] = model == "VLS128" ? 0xa1 : 0x22;
  return packet;
}

std::shared_ptr<velodyne_msgs::msg::VelodyneScan> scanWithPackets(size_t count = 1)
{
  auto scan = std::make_shared<velodyne_msgs::msg::VelodyneScan>();
  scan->header.frame_id = "lidar";
  scan->header.stamp.sec = 1;
  scan->packets.resize(count);
  return scan;
}

template<typename T>
T field(const sensor_msgs::msg::PointCloud2 & cloud, size_t point, size_t offset)
{
  T value;
  std::memcpy(&value, cloud.data.data() + point * cloud.point_step + offset, sizeof(value));
  return value;
}

TEST(CloudMemory, packed_xyzirt_round_trips_multiple_points_and_empty_scans)
{
  auto clock = std::make_shared<rclcpp::Clock>();
  velodyne_pointcloud::PointcloudXYZIRT container(0.1, 100, "", "", 384, clock);
  container.setup(scanWithPackets());
  container.addPoint(1, 2, 3, 7, 4, 5, 0.123f);
  container.addPoint(6, 7, 8, 9, 10, 11, 0.456f);
  const auto & cloud = container.finishCloud();
  EXPECT_EQ(cloud.point_step, 22U);
  EXPECT_EQ(cloud.width, 2U);
  EXPECT_EQ(cloud.data.size(), 44U);
  EXPECT_FLOAT_EQ(field<float>(cloud, 0, 18), 0.123f);
  EXPECT_FLOAT_EQ(field<float>(cloud, 1, 0), 6.0f);
  EXPECT_FLOAT_EQ(field<float>(cloud, 1, 18), 0.456f);
  EXPECT_EQ(field<uint16_t>(cloud, 1, 16), 9);
  container.setup(scanWithPackets(0));
  EXPECT_TRUE(container.finishCloud().data.empty());
}

TEST(CloudMemory, unfilled_organized_slots_are_nan_with_valid_ring_ids)
{
  auto clock = std::make_shared<rclcpp::Clock>();
  velodyne_pointcloud::OrganizedCloudXYZIRT container(0.1, 100, "", "", 16, 384, clock);
  container.setup(scanWithPackets());
  container.addPoint(1, 2, 3, 7, 4, 5, 0.123f);
  container.newLine();
  const auto & cloud = container.finishCloud();
  EXPECT_EQ(cloud.width, 16U);
  EXPECT_EQ(cloud.height, 1U);
  EXPECT_TRUE(std::isnan(field<float>(cloud, 0, 0)));
  EXPECT_TRUE(std::isnan(field<float>(cloud, 0, 18)));
  EXPECT_EQ(field<uint16_t>(cloud, 15, 16), 15);
  EXPECT_FLOAT_EQ(field<float>(cloud, 7, 0), 1.0f);
  container.setup(scanWithPackets(0));
  EXPECT_TRUE(container.finishCloud().data.empty());
}

TEST(CloudMemory, hdl64_organized_rows_fit_the_reserved_buffer)
{
  velodyne_rawdata::RawData decoder(calibrationPath("64e_utexas.yaml"), "64E");
  decoder.setParameters(0.1, 100, 0, 2 * M_PI);
  velodyne_pointcloud::OrganizedCloudXYZIRT container(
    0.1, 100, "", "", 64, decoder.scansPerPacket(), std::make_shared<rclcpp::Clock>());
  container.setup(scanWithPackets());
  EXPECT_NO_THROW(decoder.unpack(packetFor("64E"), container, rclcpp::Time(1, 0, RCL_ROS_TIME)));
  const auto & cloud = container.finishCloud();
  EXPECT_EQ(cloud.width, 64U);
  EXPECT_EQ(cloud.height, 12U);
  EXPECT_EQ(cloud.data.size(), 64U * 12U * 22U);
}

TEST(Decoding, invalid_azimuth_or_late_block_header_discards_the_whole_packet)
{
  velodyne_rawdata::RawData decoder(calibrationPath("VLP16db.yaml"), "VLP16");
  decoder.setParameters(0.1, 100, 0, 2 * M_PI);
  velodyne_pointcloud::PointcloudXYZIRT container(
    0.1, 100, "", "", 384, std::make_shared<rclcpp::Clock>());
  auto packet = packetFor("VLP16");
  packet.data[1102] = 0xff;
  packet.data[1103] = 0xff;
  container.setup(scanWithPackets());
  decoder.unpack(packet, container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(container.finishCloud().width, 0U);
  packet = packetFor("VLP16");
  packet.data[1101] = 0;
  container.setup(scanWithPackets());
  decoder.unpack(packet, container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(container.finishCloud().width, 0U);
}

TEST(Decoding, vls128_third_sequence_uses_its_own_azimuth)
{
  velodyne_rawdata::RawData decoder(calibrationPath("VLS128.yaml"), "VLS128");
  decoder.setParameters(0.1, 100, 0, 2 * M_PI);
  velodyne_pointcloud::Calibration calibration(calibrationPath("VLS128.yaml"));
  velodyne_pointcloud::PointcloudXYZIRT container(
    0.1, 100, "", "", 384, std::make_shared<rclcpp::Clock>());
  container.setup(scanWithPackets());
  decoder.unpack(packetFor("VLS128"), container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  const auto & cloud = container.finishCloud();
  ASSERT_EQ(cloud.width, 384U);
  for (size_t sequence = 0; sequence < 3; ++sequence) {
    const auto index = sequence * 128;
    const float x = field<float>(cloud, index, 0);
    const float y = field<float>(cloud, index, 4);
    const double bearing = std::atan2(-y, x) + calibration.laser_corrections[0].rot_correction;
    EXPECT_NEAR(bearing, (1000 + sequence * 40) * M_PI / 18000.0, 1e-6);
  }
}

TEST(Decoding, vls128_wrapped_view_excludes_points_outside_both_limits)
{
  velodyne_rawdata::RawData decoder(calibrationPath("VLS128.yaml"), "VLS128");
  decoder.setParameters(0.1, 100, 0, M_PI / 2);
  velodyne_pointcloud::PointcloudXYZIRT container(
    0.1, 100, "", "", 384, std::make_shared<rclcpp::Clock>());
  container.setup(scanWithPackets());
  decoder.unpack(packetFor("VLS128", 18000), container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(container.finishCloud().width, 0U);
}

TEST(Decoding, mismatched_model_and_calibration_are_rejected)
{
  EXPECT_THROW(velodyne_rawdata::RawData(calibrationPath("VLP16db.yaml"), "VLS128"),
    std::invalid_argument);
}

TEST(CalibrationValidation, rejects_bad_ids_counts_and_nonfinite_values)
{
  const auto original = YAML::LoadFile(calibrationPath("VLP16db.yaml"));
  const auto path = std::filesystem::temp_directory_path() /
    ("velodyne_calibration_validation_" + std::to_string(::getpid()) + ".yaml");
  const auto rejects = [&](YAML::Node node) {
      {std::ofstream output(path); output << node;}
      EXPECT_THROW(velodyne_pointcloud::Calibration calibration(path.string()), std::runtime_error);
    };
  auto node = YAML::Clone(original);
  node["num_lasers"] = -1;
  rejects(node);
  node = YAML::Clone(original);
  node["lasers"][0]["laser_id"] = -1;
  rejects(node);
  node = YAML::Clone(original);
  node["lasers"][1]["laser_id"] = node["lasers"][0]["laser_id"].as<int>();
  rejects(node);
  node = YAML::Clone(original);
  node["lasers"][0]["vert_correction"] = std::numeric_limits<float>::quiet_NaN();
  rejects(node);
  std::filesystem::remove(path);
}

TEST(CalibrationValidation, equal_vertical_angles_still_have_unique_rings)
{
  auto node = YAML::LoadFile(calibrationPath("VLP16db.yaml"));
  for (auto laser : node["lasers"]) {
    laser["vert_correction"] = 0.0;
  }
  const auto path = std::filesystem::temp_directory_path() /
    ("velodyne_calibration_equal_angles_" + std::to_string(::getpid()) + ".yaml");
  {std::ofstream output(path); output << node;}
  velodyne_pointcloud::Calibration calibration(path.string());
  std::filesystem::remove(path);
  std::set<int> rings;
  for (const auto & correction : calibration.laser_corrections) {
    rings.insert(correction.laser_ring);
  }
  EXPECT_EQ(rings.size(), 16U);
}
TEST(Decoding, vls128_dual_returns_have_separate_complete_rows_and_shared_firing_times)
{
  velodyne_rawdata::RawData decoder(calibrationPath("VLS128.yaml"), "VLS128");
  decoder.setParameters(0.1, 100, 0, 2 * M_PI);
  auto packet = packetFor("VLS128");
  packet.data[1204] = 57;
  for (int block = 0; block < 8; ++block) {
    packet.data[block * 100 + 1] = 0xee - 0x11 * (block / 2);
    packet.data[block * 100 + 2] = 0xe8;
    packet.data[block * 100 + 3] = 0x03;
    for (int point = 0; point < 32; ++point) {
      packet.data[block * 100 + 4 + point * 3] = block % 2 ? 0xd0 : 0xe8;
      packet.data[block * 100 + 5 + point * 3] = block % 2 ? 0x07 : 0x03;
    }
  }
  velodyne_pointcloud::OrganizedCloudXYZIRT container(
    0.1, 100, "", "", 128, 384, std::make_shared<rclcpp::Clock>());
  container.setup(scanWithPackets());
  decoder.unpack(packet, container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  const auto & cloud = container.finishCloud();
  ASSERT_EQ(cloud.width, 128U);
  ASSERT_EQ(cloud.height, 2U);
  for (size_t ring = 0; ring < 128; ++ring) {
    const float x = field<float>(cloud, ring, 0);
    const float y = field<float>(cloud, ring, 4);
    const float z = field<float>(cloud, ring, 8);
    EXPECT_NEAR(std::sqrt(x * x + y * y + z * z), 4.0f, 1e-5);
    const float second_x = field<float>(cloud, ring + 128, 0);
    const float second_y = field<float>(cloud, ring + 128, 4);
    const float second_z = field<float>(cloud, ring + 128, 8);
    EXPECT_NEAR(std::sqrt(second_x * second_x + second_y * second_y + second_z * second_z),
      8.0f, 1e-5);
    EXPECT_FLOAT_EQ(field<float>(cloud, ring, 18), field<float>(cloud, ring + 128, 18));
  }
}

TEST(Decoding, hdl32_firing_times_are_sequential_and_azimuth_is_interpolated)
{
  velodyne_rawdata::RawData decoder(calibrationPath("32db.yaml"), "32E");
  decoder.setParameters(0.1, 100, 0, 2 * M_PI);
  velodyne_pointcloud::PointcloudXYZIRT container(
    0.1, 100, "", "", 384, std::make_shared<rclcpp::Clock>());
  container.setup(scanWithPackets());
  decoder.unpack(packetFor("32E"), container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  const auto & cloud = container.finishCloud();
  ASSERT_EQ(cloud.width, 384U);
  EXPECT_NEAR(field<float>(cloud, 1, 18), 1.152e-6, 1e-12);
  EXPECT_NEAR(field<float>(cloud, 31, 18), 31 * 1.152e-6, 1e-11);
  EXPECT_NEAR(field<float>(cloud, 32, 18), 46.080e-6, 1e-11);
  EXPECT_NEAR(std::atan2(-field<float>(cloud, 31, 4), field<float>(cloud, 31, 0)),
    1031 * M_PI / 18000.0, 1e-6);
}

TEST(Decoding, vlp32_dual_returns_share_block_times_and_interpolate_against_next_pair)
{
  velodyne_rawdata::RawData decoder(calibrationPath("VeloView-VLP-32C.yaml"), "32C");
  decoder.setParameters(0.1, 100, 0, 2 * M_PI);
  velodyne_pointcloud::Calibration calibration(calibrationPath("VeloView-VLP-32C.yaml"));
  velodyne_pointcloud::PointcloudXYZIRT container(
    0.1, 100, "", "", 384, std::make_shared<rclcpp::Clock>());
  auto packet = packetFor("32C");
  packet.data[1204] = 57;
  for (int block = 0; block < 12; ++block) {
    const uint16_t angle = 1000 + (block / 2) * 40;
    packet.data[block * 100 + 2] = angle & 0xff;
    packet.data[block * 100 + 3] = angle >> 8;
  }
  container.setup(scanWithPackets());
  decoder.unpack(packet, container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  const auto & cloud = container.finishCloud();
  ASSERT_EQ(cloud.width, 384U);
  EXPECT_FLOAT_EQ(field<float>(cloud, 0, 18), field<float>(cloud, 32, 18));
  EXPECT_FLOAT_EQ(field<float>(cloud, 31, 18), field<float>(cloud, 63, 18));
  const double bearing = std::atan2(-field<float>(cloud, 31, 4), field<float>(cloud, 31, 0)) +
    calibration.laser_corrections[31].rot_correction;
  EXPECT_NEAR(bearing, 1025 * M_PI / 18000.0, 1e-6);
}

TEST(Decoding, intensity_correction_uses_raw_distance_units)
{
  auto node = YAML::LoadFile(calibrationPath("VLP16db.yaml"));
  for (auto laser : node["lasers"]) {
    laser["focal_distance"] = 13100;
    laser["focal_slope"] = 1;
  }
  const auto path = std::filesystem::temp_directory_path() /
    ("velodyne_intensity_" + std::to_string(::getpid()) + ".yaml");
  {std::ofstream output(path); output << node;}
  velodyne_rawdata::RawData decoder(path.string(), "VLP16");
  std::filesystem::remove(path);
  decoder.setParameters(0.1, 100, 0, 2 * M_PI);
  auto packet = packetFor("VLP16");
  for (int block = 0; block < 12; ++block) {
    for (int point = 0; point < 32; ++point) {
      packet.data[block * 100 + 4 + point * 3] = 0xff;
      packet.data[block * 100 + 5 + point * 3] = 0x7f;
    }
  }
  velodyne_pointcloud::PointcloudXYZIRT container(
    0.1, 100, "", "", 384, std::make_shared<rclcpp::Clock>());
  container.setup(scanWithPackets());
  decoder.unpack(packet, container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  const auto & cloud = container.finishCloud();
  ASSERT_EQ(cloud.width, 384U);
  const float factor = 1.0f - 32767.0f / 65535.0f;
  EXPECT_NEAR(field<float>(cloud, 0, 12), 20 + 256 * factor * factor, 1e-5);
}

class TransformContainer : public velodyne_rawdata::DataContainerBase
{
public:
  TransformContainer()
  : DataContainerBase(0.1, 100, "", "", 0, 1, true, 384,
      std::make_shared<rclcpp::Clock>(), 1, "x", 1, sensor_msgs::msg::PointField::FLOAT32)
  {}
  void addPoint(float x, float y, float z, uint16_t, float, float, float) override
  {
    transformPoint(x, y, z);
    result = Eigen::Vector3f(x, y, z);
  }
  void newLine() override {}
  std::string target() const {return config_.target_frame;}
  std::string fixed() const {return config_.fixed_frame;}
  void installTransforms()
  {
    sensor_frame_ = "lidar";
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_buffer_->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "fixed";
    transform.child_frame_id = "lidar";
    transform.transform.rotation.w = 1;
    transform.transform.translation.x = 1;
    tf_buffer_->setTransform(transform, "test", true);
    transform.header.frame_id = "target";
    transform.child_frame_id = "fixed";
    transform.transform.translation.x = 10;
    tf_buffer_->setTransform(transform, "test", true);
  }
  Eigen::Vector3f result;
};

TEST(TransformConfiguration, target_and_fixed_order_matches_the_constructor)
{
  TransformContainer container;
  container.configure(0.1, 100, "target", "fixed");
  EXPECT_EQ(container.target(), "target");
  EXPECT_EQ(container.fixed(), "fixed");
  container.installTransforms();
  ASSERT_TRUE(container.computeTransformToTarget(rclcpp::Time(1, 0, RCL_ROS_TIME)));
  ASSERT_TRUE(container.computeTransformToFixed(rclcpp::Time(1, 0, RCL_ROS_TIME)));
  container.addPoint(1, 0, 0, 0, 1, 1, 0);
  EXPECT_FLOAT_EQ(container.result.x(), 12);
}

TEST(TransformConfiguration, equal_sensor_and_target_frames_do_not_erase_configuration)
{
  TransformContainer container;
  container.configure(0.1, 100, "lidar", "");
  container.setup(scanWithPackets());
  EXPECT_EQ(container.target(), "lidar");
  EXPECT_TRUE(container.computeTransformToTarget(rclcpp::Time(1, 0, RCL_ROS_TIME)));
}
TEST(Decoding, zero_width_view_is_empty)
{
  velodyne_rawdata::RawData decoder(calibrationPath("VLP16db.yaml"), "VLP16");
  decoder.setParameters(0.1, 100, 0, 0);
  velodyne_pointcloud::PointcloudXYZIRT container(
    0.1, 100, "", "", 384, std::make_shared<rclcpp::Clock>());
  container.setup(scanWithPackets());
  decoder.unpack(packetFor("VLP16"), container, rclcpp::Time(1, 0, RCL_ROS_TIME));
  EXPECT_EQ(container.finishCloud().width, 0U);
}
}  // namespace
