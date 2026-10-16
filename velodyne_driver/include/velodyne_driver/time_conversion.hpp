// Copyright 2019 Matthew Pitropov, Joshua Whitley
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

#ifndef VELODYNE_DRIVER__TIME_CONVERSION_HPP_
#define VELODYNE_DRIVER__TIME_CONVERSION_HPP_

#include <rclcpp/time.hpp>

/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. Velodyne only returns time since the top of the hour, so if the computer clock
 * and the velodyne clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 *
 * @param stamp timestamp recovered from velodyne
 * @param nominal_stamp time coming from computer's clock
 * @return timestamp from velodyne, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
inline
rclcpp::Time resolveHourAmbiguity(const rclcpp::Time & stamp, const rclcpp::Time & nominal_stamp)
{
  const auto difference = nominal_stamp.nanoseconds() - stamp.nanoseconds();
  const auto hour = rclcpp::Duration::from_seconds(3600.0);
  if (difference > hour.nanoseconds() / 2) {
    return stamp + hour;
  }
  if (difference < -hour.nanoseconds() / 2) {
    return stamp - hour;
  }
  return stamp;
}

inline
rclcpp::Time rosTimeFromGpsTimestamp(const rclcpp::Time & time_nom, const uint8_t * const data)
{
  // The sensor stores little-endian microseconds since the top of the hour.
  const uint32_t usecs = static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
  constexpr int64_t hour_ns = 3600LL * 1000000000LL;
  const int64_t hour_start = (time_nom.nanoseconds() / hour_ns) * hour_ns;
  const rclcpp::Time stamp(
    hour_start + static_cast<int64_t>(usecs) * 1000, time_nom.get_clock_type());
  return resolveHourAmbiguity(stamp, time_nom);
}

#endif  // VELODYNE_DRIVER__TIME_CONVERSION_HPP_
