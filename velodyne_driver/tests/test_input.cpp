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
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "velodyne_driver/driver.hpp"
#include "velodyne_driver/input.hpp"

namespace
{
using namespace std::chrono_literals;

void putNetwork16(std::vector<uint8_t> & bytes, size_t offset, uint16_t value)
{
  bytes[offset] = value >> 8;
  bytes[offset + 1] = value & 0xff;
}

std::vector<uint8_t> makeFrame(bool vlan = false, bool options = false, uint16_t port = 2368)
{
  const size_t ip_offset = vlan ? 18 : 14;
  const size_t ip_size = options ? 24 : 20;
  std::vector<uint8_t> frame(ip_offset + ip_size + 8 + 1206, 0);
  putNetwork16(frame, 12, vlan ? 0x8100 : 0x0800);
  if (vlan) {
    putNetwork16(frame, 16, 0x0800);
  }
  frame[ip_offset] = options ? 0x46 : 0x45;
  frame[ip_offset + 9] = 17;
  putNetwork16(frame, ip_offset + 2, ip_size + 8 + 1206);
  frame[ip_offset + 12] = 192;
  frame[ip_offset + 13] = 168;
  frame[ip_offset + 14] = 1;
  frame[ip_offset + 15] = 201;
  const size_t udp_offset = ip_offset + ip_size;
  putNetwork16(frame, udp_offset + 2, port);
  putNetwork16(frame, udp_offset + 4, 1214);
  frame[udp_offset + 8] = 0xff;
  frame[udp_offset + 9] = 0xee;
  frame.back() = 0x22;
  return frame;
}

class Capture
{
public:
  explicit Capture(const std::vector<std::vector<uint8_t>> & frames, int link = DLT_EN10MB)
  {
    char filename[] = "/tmp/velodyne_input_test_XXXXXX";
    const int descriptor = ::mkstemp(filename);
    if (descriptor < 0) {
      throw std::runtime_error("mkstemp failed");
    }
    ::close(descriptor);
    path = filename;
    pcap_t * context = pcap_open_dead(link, 65535);
    pcap_dumper_t * dump = pcap_dump_open(context, path.c_str());
    if (!dump) {
      pcap_close(context);
      throw std::runtime_error("pcap_dump_open failed");
    }
    for (const auto & frame : frames) {
      pcap_pkthdr header{};
      header.caplen = frame.size();
      header.len = frame.size();
      pcap_dump(reinterpret_cast<u_char *>(dump), &header, frame.data());
    }
    pcap_dump_close(dump);
    pcap_close(context);
  }
  ~Capture() {std::filesystem::remove(path);}
  std::string path;
};

uint16_t unusedPort()
{
  const int descriptor = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (descriptor < 0) {
    throw std::runtime_error("UDP socket creation failed");
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(descriptor, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    ::close(descriptor);
    throw std::runtime_error("UDP bind failed");
  }
  socklen_t length = sizeof(address);
  ::getsockname(descriptor, reinterpret_cast<sockaddr *>(&address), &length);
  ::close(descriptor);
  return ntohs(address.sin_port);
}

class InputTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {node = std::make_shared<rclcpp::Node>("input_test");}
  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(InputTest, missing_and_unsupported_captures_fail_at_construction)
{
  EXPECT_THROW(velodyne_driver::InputPCAP(node.get(), "", 2368, 754,
    "/tmp/velodyne_missing_capture.pcap", true, true, 0), std::runtime_error);
  Capture capture({}, DLT_RAW);
  EXPECT_THROW(velodyne_driver::InputPCAP(node.get(), "", 2368, 754,
    capture.path, true, true, 0), std::invalid_argument);
}

TEST_F(InputTest, empty_capture_and_repeated_eof_are_safe)
{
  Capture capture({});
  velodyne_driver::InputPCAP input(node.get(), "", 2368, 754, capture.path, true, true, 0);
  velodyne_msgs::msg::VelodynePacket packet;
  EXPECT_EQ(input.getPacket(&packet, 0), -1);
  EXPECT_EQ(input.getPacket(&packet, 0), -1);
}

TEST_F(InputTest, validates_lengths_and_filters_before_copying)
{
  auto truncated = makeFrame();
  truncated.resize(44);
  auto fragmented = makeFrame();
  fragmented[20] = 0x20;
  auto wrong_length = makeFrame();
  putNetwork16(wrong_length, 38, 40);
  Capture capture({truncated, fragmented, wrong_length, makeFrame(false, false, 1000),
      makeFrame()});
  velodyne_driver::InputPCAP input(
    node.get(), "192.168.1.201", 2368, 754, capture.path, true, true, 0);
  velodyne_msgs::msg::VelodynePacket packet;
  const auto before = node->now();
  ASSERT_EQ(input.getPacket(&packet, 0.25), 0);
  EXPECT_EQ(packet.data[0], 0xff);
  EXPECT_EQ(packet.data[1], 0xee);
  EXPECT_EQ(packet.data.back(), 0x22);
  EXPECT_GE(rclcpp::Time(packet.stamp).nanoseconds(), before.nanoseconds() + 250000000);
  EXPECT_LE(rclcpp::Time(packet.stamp).nanoseconds(), node->now().nanoseconds() + 250000000);
  EXPECT_EQ(input.getPacket(&packet, 0), -1);
}

TEST_F(InputTest, supports_vlan_and_ipv4_options)
{
  Capture capture({makeFrame(true, true)});
  velodyne_driver::InputPCAP input(node.get(), "", 2368, 754, capture.path, true, true, 0);
  velodyne_msgs::msg::VelodynePacket packet;
  ASSERT_EQ(input.getPacket(&packet, 0), 0);
  EXPECT_EQ(packet.data[0], 0xff);
  EXPECT_EQ(packet.data.back(), 0x22);
}

TEST_F(InputTest, repeat_delay_is_interruptible)
{
  Capture capture({makeFrame()});
  velodyne_driver::InputPCAP input(node.get(), "", 2368, 754, capture.path, false, true, 60);
  velodyne_msgs::msg::VelodynePacket packet;
  ASSERT_EQ(input.getPacket(&packet, 0), 0);
  auto result = std::async(std::launch::async, [&]() {return input.getPacket(&packet, 0);});
  std::this_thread::sleep_for(30ms);
  input.requestStop();
  EXPECT_EQ(result.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(result.get(), -1);
}

TEST_F(InputTest, missing_capture_on_repeat_is_safe)
{
  Capture capture({makeFrame()});
  velodyne_driver::InputPCAP input(node.get(), "", 2368, 754, capture.path, false, true, 0);
  velodyne_msgs::msg::VelodynePacket packet;
  ASSERT_EQ(input.getPacket(&packet, 0), 0);
  std::filesystem::remove(capture.path);
  EXPECT_EQ(input.getPacket(&packet, 0), -1);
}

TEST_F(InputTest, socket_timeout_is_retryable_and_stop_is_bounded)
{
  velodyne_driver::InputSocket input(node.get(), "", unusedPort(), false);
  velodyne_msgs::msg::VelodynePacket packet;
  EXPECT_GT(input.getPacket(&packet, 0), 0);
  input.requestStop();
  EXPECT_EQ(input.getPacket(&packet, 0), -1);
}

TEST_F(InputTest, rejects_oversized_udp_datagrams_and_applies_seconds_offset)
{
  const auto port = unusedPort();
  velodyne_driver::InputSocket input(node.get(), "", port, false);
  const int sender = ::socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(sender, 0);
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  std::vector<uint8_t> bytes(1207, 0xaa);
  ASSERT_EQ(::sendto(sender, bytes.data(), bytes.size(), 0,
    reinterpret_cast<sockaddr *>(&destination), sizeof(destination)), 1207);
  bytes.resize(1206);
  bytes[0] = 0xff;
  ASSERT_EQ(::sendto(sender, bytes.data(), bytes.size(), 0,
    reinterpret_cast<sockaddr *>(&destination), sizeof(destination)), 1206);
  ::close(sender);
  velodyne_msgs::msg::VelodynePacket packet;
  const auto before = node->now();
  ASSERT_EQ(input.getPacket(&packet, 0.5), 0);
  EXPECT_EQ(packet.data[0], 0xff);
  EXPECT_GE(rclcpp::Time(packet.stamp).nanoseconds(), before.nanoseconds() + 500000000);
}

TEST_F(InputTest, driver_unloads_without_a_sensor_and_marks_startup_parameters_read_only)
{
  auto options = rclcpp::NodeOptions().parameter_overrides({
      {"model", "VLP16"}, {"port", static_cast<int>(unusedPort())}});
  auto driver = std::make_unique<velodyne_driver::VelodyneDriver>(options);
  EXPECT_FALSE(driver->set_parameter(rclcpp::Parameter("rpm", 1200.0)).successful);
  EXPECT_TRUE(driver->set_parameter(rclcpp::Parameter("time_offset", 0.25)).successful);
  EXPECT_FALSE(driver->set_parameter(rclcpp::Parameter(
      "time_offset", std::numeric_limits<double>::quiet_NaN())).successful);
  EXPECT_TRUE(driver->set_parameter(rclcpp::Parameter("enabled", false)).successful);
  const auto start = std::chrono::steady_clock::now();
  driver.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
}

TEST_F(InputTest, validates_driver_configuration)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides({{"rpm", 0.0}});
  EXPECT_THROW(velodyne_driver::VelodyneDriver driver(options), std::invalid_argument);
}
TEST_F(InputTest, fixed_packet_scans_cover_a_full_dual_return_revolution)
{
  auto frame = makeFrame();
  frame[42 + 1204] = 57;
  Capture capture(std::vector<std::vector<uint8_t>>(152, frame));
  auto driver_options = rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({
      {"model", "VLP16"}, {"pcap", capture.path}, {"enabled", false},
      {"read_once", true}, {"read_fast", true}});
  auto driver = std::make_shared<velodyne_driver::VelodyneDriver>(driver_options);
  auto observer = std::make_shared<rclcpp::Node>(
    "scan_observer", rclcpp::NodeOptions().use_intra_process_comms(true));
  velodyne_msgs::msg::VelodyneScan::ConstSharedPtr received;
  auto subscription = observer->create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    [&](velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan) {received = scan;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(driver);
  ASSERT_TRUE(driver->set_parameter(rclcpp::Parameter("enabled", true)).successful);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(received->packets.size(), 152U);
}

TEST_F(InputTest, cut_angle_two_pi_is_equivalent_to_zero)
{
  std::vector<std::vector<uint8_t>> frames;
  for (const uint16_t angle : {35900, 35950, 10}) {
    auto frame = makeFrame();
    frame[44] = angle & 0xff;
    frame[45] = angle >> 8;
    frames.push_back(frame);
  }
  Capture capture(frames);
  auto driver = std::make_shared<velodyne_driver::VelodyneDriver>(
    rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({
      {"model", "VLP16"}, {"pcap", capture.path}, {"enabled", false},
      {"read_once", true}, {"read_fast", true}, {"cut_angle", 2 * M_PI}}));
  auto observer = std::make_shared<rclcpp::Node>(
    "cut_observer", rclcpp::NodeOptions().use_intra_process_comms(true));
  velodyne_msgs::msg::VelodyneScan::ConstSharedPtr received;
  auto subscription = observer->create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    [&](velodyne_msgs::msg::VelodyneScan::ConstSharedPtr scan) {received = scan;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  executor.add_node(driver);
  ASSERT_TRUE(driver->set_parameter(rclcpp::Parameter("enabled", true)).successful);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(received);
  EXPECT_EQ(received->packets.size(), 3U);
}
}  // namespace
