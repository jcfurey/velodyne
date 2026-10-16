// Copyright 2007, 2009, 2010, 2015, 2019 Austin Robot Technology, Patrick Beeson, Jack O'Quin, AutonomouStuff  // NOLINT
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

/** \file
 *
 *  Input classes for the Velodyne HDL-64E 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include "velodyne_driver/input.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include "velodyne_driver/time_conversion.hpp"

namespace velodyne_driver
{
namespace
{
constexpr size_t packet_size = 1206;

uint16_t readNetwork16(const u_char * data)
{
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

// Locate a complete, unfragmented IPv4/UDP payload in an Ethernet capture.
// Capture lengths and IP/UDP lengths must all agree before any copy.
const u_char * packetPayload(
  const pcap_pkthdr & header, const u_char * data, size_t & ip_offset)
{
  ip_offset = 14;
  if (header.caplen < ip_offset) {
    return nullptr;
  }
  uint16_t protocol = readNetwork16(data + 12);
  while (protocol == 0x8100 || protocol == 0x88a8) {
    if (header.caplen < ip_offset + 4) {
      return nullptr;
    }
    protocol = readNetwork16(data + ip_offset + 2);
    ip_offset += 4;
  }
  if (protocol != 0x0800 || header.caplen < ip_offset + 20) {
    return nullptr;
  }
  const u_char * ip = data + ip_offset;
  const size_t ip_size = (ip[0] & 0x0f) * 4;
  if ((ip[0] >> 4) != 4 || ip_size < 20 || ip[9] != 17 ||
    (readNetwork16(ip + 6) & 0x3fff) != 0 ||
    header.caplen < ip_offset + ip_size + 8)
  {
    return nullptr;
  }
  const u_char * udp = ip + ip_size;
  const size_t udp_size = readNetwork16(udp + 4);
  const size_t total_size = readNetwork16(ip + 2);
  if (udp_size != packet_size + 8 || total_size != ip_size + udp_size ||
    header.caplen < ip_offset + total_size)
  {
    return nullptr;
  }
  return udp + 8;
}
}  // namespace

Input::Input(rclcpp::Node * private_nh, const std::string & devip, uint16_t port)
: private_nh_(private_nh), devip_str_(devip), port_(port)
{
  in_addr address{};
  if (!devip.empty() && ::inet_pton(AF_INET, devip.c_str(), &address) != 1) {
    throw std::invalid_argument("device_ip must be a numeric IPv4 address");
  }
}

void Input::requestStop()
{
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_requested_.store(true);
  }
  stop_cv_.notify_all();
}

bool Input::stopped() const
{
  return stop_requested_.load() ||
         !rclcpp::ok(private_nh_->get_node_base_interface()->get_context());
}

bool Input::waitUntil(std::chrono::steady_clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(stop_mutex_);
  while (!stopped()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return true;
    }
    stop_cv_.wait_until(lock, std::min(deadline, now + std::chrono::milliseconds(100)));
  }
  return false;
}

InputSocket::InputSocket(
  rclcpp::Node * private_nh, const std::string & devip, uint16_t port, bool gps_time)
: Input(private_nh, devip, port), sockfd_(-1), gps_time_(gps_time)
{
  if (!devip.empty()) {
    ::inet_pton(AF_INET, devip.c_str(), &devip_);
  }
  RCLCPP_INFO(private_nh->get_logger(), "Opening UDP socket: port %u", port);
  sockfd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (sockfd_ == -1) {
    throw std::runtime_error("Error opening socket: " + std::string(::strerror(errno)));
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = INADDR_ANY;
  int reuse = 1;
  if (::setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1 ||
    ::bind(sockfd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == -1)
  {
    const std::string error = ::strerror(errno);
    ::close(sockfd_);
    sockfd_ = -1;
    throw std::runtime_error("Error configuring UDP socket: " + error);
  }
}

InputSocket::~InputSocket()
{
  if (sockfd_ >= 0) {
    ::close(sockfd_);
  }
}

int InputSocket::getPacket(velodyne_msgs::msg::VelodynePacket * pkt, const double time_offset)
{
  while (!stopped()) {
    pollfd descriptor{sockfd_, POLLIN, 0};
    const int result = ::poll(&descriptor, 1, 100);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(private_nh_->get_logger(), "poll() error: %s", ::strerror(errno));
      return -1;
    }
    if (result == 0) {
      return 1;
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      RCLCPP_ERROR(private_nh_->get_logger(), "poll() reports Velodyne error");
      return -1;
    }
    if (!(descriptor.revents & POLLIN)) {
      continue;
    }
    sockaddr_in sender{};
    socklen_t sender_size = sizeof(sender);
    const auto time1 = private_nh_->now();
    // MSG_TRUNC returns the actual datagram length, rejecting oversized packets.
    const ssize_t bytes = ::recvfrom(
      sockfd_, pkt->data.data(), pkt->data.size(), MSG_TRUNC,
      reinterpret_cast<sockaddr *>(&sender), &sender_size);
    const auto time2 = private_nh_->now();
    if (bytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(private_nh_->get_logger(), "recvfrom() error: %s", ::strerror(errno));
      return -1;
    }
    if (bytes != static_cast<ssize_t>(packet_size) ||
      (!devip_str_.empty() && sender.sin_addr.s_addr != devip_.s_addr))
    {
      continue;
    }
    const auto stamp = gps_time_ ? rosTimeFromGpsTimestamp(time2, pkt->data.data() + 1200) :
      rclcpp::Time(
      time1.nanoseconds() + (time2.nanoseconds() - time1.nanoseconds()) / 2,
      time2.get_clock_type());
    pkt->stamp = stamp + rclcpp::Duration::from_seconds(time_offset);
    return 0;
  }
  return -1;
}

InputPCAP::InputPCAP(
  rclcpp::Node * private_nh, const std::string & devip, uint16_t port,
  double packet_rate, const std::string & filename, bool read_once,
  bool read_fast, double repeat_delay)
: Input(private_nh, devip, port), filename_(filename),
  read_once_(read_once), read_fast_(read_fast), repeat_delay_(repeat_delay)
{
  if (!std::isfinite(packet_rate) || packet_rate <= 0.0 ||
    !std::isfinite(repeat_delay) || repeat_delay < 0.0 || repeat_delay > 86400.0)
  {
    throw std::invalid_argument("Invalid PCAP packet rate or repeat delay");
  }
  packet_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / packet_rate));
  next_packet_time_ = std::chrono::steady_clock::now();
  pcap_.reset(pcap_open_offline(filename.c_str(), errbuf_));
  if (!pcap_) {
    throw std::runtime_error("Error opening PCAP file: " + std::string(errbuf_));
  }
  if (pcap_datalink(pcap_.get()) != DLT_EN10MB) {
    throw std::invalid_argument("PCAP input requires Ethernet link-layer frames");
  }
  // Apply the filter to the untagged IP frame after parsing VLAN headers.
  std::unique_ptr<pcap_t, decltype(& pcap_close)> filter_context(
    pcap_open_dead(DLT_RAW, 65535), &pcap_close);
  if (!filter_context) {
    throw std::runtime_error("Could not create PCAP filter context");
  }
  std::stringstream filter;
  if (!devip.empty()) {
    filter << "src host " << devip << " and ";
  }
  filter << "udp dst port " << port;
  if (pcap_compile(
      filter_context.get(), &pcap_packet_filter_, filter.str().c_str(), 1,
      PCAP_NETMASK_UNKNOWN) < 0)
  {
    throw std::runtime_error("Error compiling PCAP filter: " +
            std::string(pcap_geterr(filter_context.get())));
  }
  filter_initialized_ = true;
}

InputPCAP::~InputPCAP()
{
  if (filter_initialized_) {
    pcap_freecode(&pcap_packet_filter_);
  }
}

int InputPCAP::getPacket(velodyne_msgs::msg::VelodynePacket * pkt, const double time_offset)
{
  while (!stopped() && !exhausted_) {
    pcap_pkthdr * header = nullptr;
    const u_char * frame = nullptr;
    const int result = pcap_next_ex(pcap_.get(), &header, &frame);
    if (result == 0) {
      continue;
    }
    if (result == 1) {
      size_t ip_offset = 0;
      const auto payload = packetPayload(*header, frame, ip_offset);
      if (!payload) {
        continue;
      }
      pcap_pkthdr ip_header = *header;
      ip_header.caplen -= ip_offset;
      ip_header.len = ip_header.caplen;
      if (!pcap_offline_filter(&pcap_packet_filter_, &ip_header, frame + ip_offset)) {
        continue;
      }
      if (!read_fast_) {
        const auto now = std::chrono::steady_clock::now();
        next_packet_time_ = std::max(next_packet_time_ + packet_period_, now);
        if (!waitUntil(next_packet_time_)) {
          return -1;
        }
      }
      std::memcpy(pkt->data.data(), payload, packet_size);
      pkt->stamp = private_nh_->now() + rclcpp::Duration::from_seconds(time_offset);
      empty_ = false;
      return 0;
    }
    if (result == -1) {
      RCLCPP_ERROR(private_nh_->get_logger(), "PCAP read error: %s", pcap_geterr(pcap_.get()));
    }
    if (result == -1 || empty_ || read_once_) {
      exhausted_ = true;
      return -1;
    }
    if (!waitUntil(std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(repeat_delay_))))
    {
      return -1;
    }
    pcap_.reset(pcap_open_offline(filename_.c_str(), errbuf_));
    if (!pcap_ || pcap_datalink(pcap_.get()) != DLT_EN10MB) {
      RCLCPP_ERROR(private_nh_->get_logger(), "Could not reopen Ethernet PCAP: %s", errbuf_);
      exhausted_ = true;
      return -1;
    }
    empty_ = true;
  }
  return -1;
}

}  // namespace velodyne_driver
