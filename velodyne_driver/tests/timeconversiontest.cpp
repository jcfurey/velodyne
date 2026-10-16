// Copyright 2019 Matthew Pitropov, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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

#include <rclcpp/rclcpp.hpp>

#include "velodyne_driver/time_conversion.hpp"

TEST(TimeConversion, BytesToTimestamp)
{
  // Use an exact microsecond timestamp and 64-bit arithmetic so the test is
  // deterministic and remains valid near the end of an hour and after 2038.
  const rclcpp::Time ros_stamp(2208988799123456000LL, RCL_ROS_TIME);
  const uint32_t since_the_hour = (ros_stamp.nanoseconds() / 1000) % 3600000000ULL;

  uint8_t native_format[4];
  native_format[0] = 0xFF & since_the_hour;
  native_format[1] = 0xFF & (((uint32_t)since_the_hour) >> 8);
  native_format[2] = 0xFF & (((uint32_t)since_the_hour) >> 16);
  native_format[3] = 0xFF & (((uint32_t)since_the_hour) >> 24);

  rclcpp::Time ros_stamp_converted = rosTimeFromGpsTimestamp(ros_stamp, native_format);

  ASSERT_EQ(ros_stamp_converted.nanoseconds(), ros_stamp.nanoseconds());
}

TEST(TimeConversion, preserves_nanoseconds_and_clock_at_next_hour)
{
  const rclcpp::Time nominal(3600LL * 1000000000 + 100000000, RCL_ROS_TIME);
  const rclcpp::Time previous(3600LL * 1000000000 - 123456789, RCL_ROS_TIME);
  const rclcpp::Time wrong_hour(previous.nanoseconds() + 3600LL * 1000000000, RCL_ROS_TIME);
  const auto converted = resolveHourAmbiguity(wrong_hour, nominal);
  EXPECT_EQ(converted.nanoseconds(), previous.nanoseconds());
  EXPECT_EQ(converted.get_clock_type(), RCL_ROS_TIME);
}

TEST(TimeConversion, preserves_nanoseconds_and_clock_at_previous_hour)
{
  const rclcpp::Time nominal(7200LL * 1000000000 - 100000000, RCL_ROS_TIME);
  const rclcpp::Time next(7200LL * 1000000000 + 123456789, RCL_ROS_TIME);
  const rclcpp::Time wrong_hour(next.nanoseconds() - 3600LL * 1000000000, RCL_ROS_TIME);
  EXPECT_EQ(resolveHourAmbiguity(wrong_hour, nominal).nanoseconds(), next.nanoseconds());
}

TEST(TimeConversion, gps_timestamp_keeps_ros_clock_and_microsecond_precision)
{
  const rclcpp::Time nominal(3600LL * 1000000000 + 123456000, RCL_ROS_TIME);
  const uint32_t micros = 123456;
  const uint8_t bytes[] = {
    static_cast<uint8_t>(micros), static_cast<uint8_t>(micros >> 8),
    static_cast<uint8_t>(micros >> 16), static_cast<uint8_t>(micros >> 24)};
  const auto converted = rosTimeFromGpsTimestamp(nominal, bytes);
  EXPECT_EQ(converted.nanoseconds(), nominal.nanoseconds());
  EXPECT_EQ(converted.get_clock_type(), nominal.get_clock_type());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
