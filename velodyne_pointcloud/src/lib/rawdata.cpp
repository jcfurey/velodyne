// Copyright 2007, 2009, 2010, 2012, 2019 Austin Robot Technology, Patrick Beeson, Jack O'Quin, Kaarta Inc, Shawn Hanna, Joshua Whitley  // NOLINT
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

#include <angles/angles.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <cassert>

#include <velodyne_msgs/msg/velodyne_packet.hpp>

#include "velodyne_pointcloud/datacontainerbase.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace velodyne_rawdata
{

namespace
{
uint16_t readLittle16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

raw_packet decodePacket(const velodyne_msgs::msg::VelodynePacket & pkt)
{
  raw_packet decoded{};
  for (int block = 0; block < BLOCKS_PER_PACKET; ++block) {
    const auto * source = pkt.data.data() + block * SIZE_BLOCK;
    decoded.blocks[block].header = readLittle16(source);
    decoded.blocks[block].rotation = readLittle16(source + 2);
    std::memcpy(decoded.blocks[block].data, source + 4, BLOCK_DATA_SIZE);
  }
  return decoded;
}
}  // namespace

inline float square(float val)
{
  return val * val;
}

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData(const std::string & calibration_file, const std::string & model)
{
  calibration_ = std::make_unique<velodyne_pointcloud::Calibration>(calibration_file);
  config_.model = model;
  const int expected_lasers = model == "VLP16" ? 16 :
    (model == "32C" || model == "32E") ? 32 : model == "VLS128" ? 128 :
    (model == "64E" || model == "64E_S2" || model == "64E_S2.1" || model == "64E_S3") ? 64 : 0;
  if (expected_lasers == 0 || calibration_->num_lasers != expected_lasers) {
    throw std::invalid_argument("Calibration laser count does not match the configured model");
  }
  buildTimings();

  // RCLCPP_INFO(this->get_logger(), "Number of lasers: %d.",  calibration_->num_lasers);

  setupSinCosCache();
  if (config_.model == "VLS128") {
    setupAzimuthCache();
  }
}

/** Update parameters: conversions and update */
void RawData::setParameters(
  double min_range, double max_range, double view_direction, double view_width)
{
  if (!std::isfinite(min_range) || !std::isfinite(max_range) || min_range < 0.0 ||
    min_range > max_range || !std::isfinite(view_direction) ||
    !std::isfinite(view_width) || view_width < 0.0 || view_width > 2.0 * M_PI)
  {
    throw std::invalid_argument("Invalid range or view configuration");
  }
  config_.min_range = min_range;
  config_.max_range = max_range;

  // converting angle parameters into the velodyne reference (rad)
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  tmp_min_angle = ::fmod(::fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = ::fmod(::fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware velodyne ref (negative yaml and degrees)
  // adding 0.5 performs a centered double to int conversion
  config_.min_angle = std::lround(100.0 * (2.0 * M_PI - tmp_min_angle) * 180.0 / M_PI);
  config_.max_angle = std::lround(100.0 * (2.0 * M_PI - tmp_max_angle) * 180.0 / M_PI);

  if (view_width == 2.0 * M_PI) {
    // Full revolutions have identical normalized endpoints.
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

void RawData::setVlp16DualReturnMode(const std::string & mode)
{
  if (mode == "both") {
    vlp16_dual_return_mode_ = Vlp16DualReturnMode::Both;
  } else if (mode == "strongest") {
    vlp16_dual_return_mode_ = Vlp16DualReturnMode::Strongest;
  } else if (mode == "last") {
    vlp16_dual_return_mode_ = Vlp16DualReturnMode::Last;
  } else {
    throw std::invalid_argument(
            "vlp16_dual_return_mode must be one of: both, strongest, last");
  }
}

void RawData::setVlp16ScanBoundaryClipping(bool enabled)
{
  clip_vlp16_scan_boundaries_ = enabled;
}

void RawData::setVlp16PacketTimestampReference(int block_reference)
{
  if (block_reference < 0 || block_reference >= BLOCKS_PER_PACKET) {
    throw std::invalid_argument(
            "vlp16_packet_timestamp_reference must be in [0, 11]");
  }
  vlp16_packet_timestamp_reference_ = block_reference;
}

int RawData::scansPerPacket() const
{
  if (calibration_->num_lasers == 16) {
    return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK *
           VLP16_SCANS_PER_FIRING;
  }
  return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
}

/**
 * Build a timing table for each block/firing. Stores in timing_offsets vector
 */
bool RawData::buildTimings()
{
  // vlp16
  if (config_.model == "VLP16") {
    // timing table calculation, from velodyne user manual
    timing_offsets_.resize(12);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6;   // seconds
    double single_firing = 2.304 * 1e-6;   // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = (x - (x % 2)) + (y / 16);
        } else {
          dataBlockIndex = (x * 2) + (y / 16);
        }
        dataPointIndex = y % 16;
        // timing_offsets[block][firing]
        timing_offsets_[x][y] = (full_firing_cycle * dataBlockIndex) +
          (single_firing * dataPointIndex);
      }
    }
    // vlp32
  } else if (config_.model == "32C") {
    // timing table calculation, from velodyne user manual
    timing_offsets_.resize(12);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6;   // seconds
    double single_firing = 2.304 * 1e-6;   // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = x / 2;
        } else {
          dataBlockIndex = x;
        }
        dataPointIndex = y / 2;
        timing_offsets_[x][y] = (full_firing_cycle * dataBlockIndex) +
          (single_firing * dataPointIndex);
      }
    }
    // hdl32
  } else if (config_.model == "32E") {
    // timing table calculation, from velodyne user manual
    timing_offsets_.resize(12);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(32);
    }
    // constants
    double full_firing_cycle = 46.080 * 1e-6;   // seconds
    double single_firing = 1.152 * 1e-6;   // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = x / 2;
        } else {
          dataBlockIndex = x;
        }
        dataPointIndex = y;
        timing_offsets_[x][y] = (full_firing_cycle * dataBlockIndex) +
          (single_firing * dataPointIndex);
      }
    }
  } else if (config_.model == "VLS128") {
    timing_offsets_.resize(3);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(17);  // 17 (+1 for the maintenance time after firing group 8)
    }

    double full_firing_cycle = VLS128_SEQ_TDURATION * 1e-6;   // seconds
    double single_firing = VLS128_CHANNEL_TDURATION * 1e-6;   // seconds
    double offset_paket_time = VLS128_TOH_ADJUSTMENT * 1e-6;  // seconds
    double sequenceIndex, firingGroupIndex;
    // Compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        sequenceIndex = x;
        firingGroupIndex = y;
        timing_offsets_[x][y] =
          (full_firing_cycle * sequenceIndex) + (single_firing * firingGroupIndex) -
          offset_paket_time;
        RCLCPP_DEBUG(
          rclcpp::get_logger("velodyne_pointcloud"),
          "firing_seque %lu firing_group %lu offset %f", x, y, timing_offsets_[x][y]);
      }
    }
  } else {
    timing_offsets_.clear();
    RCLCPP_WARN(
      rclcpp::get_logger("velodyne_pointcloud"),
      "Timings not supported for model %s", config_.model.c_str());
  }

  if (timing_offsets_.size()) {
    return true;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("velodyne_pointcloud"),
      "NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED VELODYNE SENSOR?");
  }
  return false;
}

int RawData::numLasers() const
{
  return calibration_->num_lasers;
}

void RawData::setupSinCosCache()
{
  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = calibration_->correctedAzimuth(
      angles::from_degrees(ROTATION_RESOLUTION * rot_index));
    cos_rot_table_[rot_index] = ::cosf(rotation);
    sin_rot_table_[rot_index] = ::sinf(rotation);
  }
}

void RawData::setupAzimuthCache()
{
  if (config_.model == "VLS128") {
    for (uint8_t i = 0; i < 16; i++) {
      vls_128_laser_azimuth_cache_[i] =
        (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
    }
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("velodyne_pointcloud"),
      "No Azimuth Cache configured for model %s", config_.model.c_str());
  }
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(
  const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data,
  const rclcpp::Time & scan_start_time, bool is_first_packet, bool is_last_packet)
{
  const raw_packet decoded = decodePacket(pkt);
  const raw_packet * raw = &decoded;
  const bool is_vls128 = config_.model == "VLS128";
  if ((pkt.data[1205] == VLS128_MODEL_ID) != is_vls128) {
    return;
  }
  const int blocks = is_vls128 && pkt.data[1204] == 57 ? 8 : BLOCKS_PER_PACKET;
  for (int block = 0; block < blocks; ++block) {
    const auto & current = raw->blocks[block];
    const uint16_t vls_banks[] = {VLS128_BANK_1, VLS128_BANK_2, VLS128_BANK_3, VLS128_BANK_4};
    const int bank_index = pkt.data[1204] == 57 ? block / 2 : block;
    const bool valid_bank = is_vls128 ? current.header == vls_banks[bank_index % 4] :
      (current.header == UPPER_BANK ||
      (calibration_->num_lasers == 64 && current.header == LOWER_BANK));
    if (!valid_bank || current.rotation >= ROTATION_MAX_UNITS) {
      return;
    }
  }
  if (is_vls128) {
    unpack_vls128(pkt, data, scan_start_time, decoded);
    return;
  }
  if (calibration_->num_lasers == 16) {
    unpack_vlp16(pkt, data, scan_start_time, is_first_packet, is_last_packet, decoded);
    return;
  }

  float time_diff_start_to_this_packet =
    (rclcpp::Time(pkt.stamp) - scan_start_time).seconds();

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    // upper bank lasers are numbered [0..31]
    // NOTE: this is a change from the old velodyne_common implementation

    int bank_origin = 0;

    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }

    const int block_step = pkt.data[1204] == 57 ? 2 : 1;
    int azimuth_delta = 0;
    if (calibration_->num_lasers == 32) {
      const int adjacent = i + block_step < BLOCKS_PER_PACKET ? i + block_step : i - block_step;
      const int delta = adjacent > i ? raw->blocks[adjacent].rotation - raw->blocks[i].rotation :
        raw->blocks[i].rotation - raw->blocks[adjacent].rotation;
      azimuth_delta = (ROTATION_MAX_UNITS + delta) % ROTATION_MAX_UNITS;
      if (azimuth_delta > ROTATION_MAX_UNITS / 2) {
        azimuth_delta = 0;
      }
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      float x, y, z;
      float intensity;
      const uint8_t laser_number = j + bank_origin;

      const velodyne_pointcloud::LaserCorrection & corrections =
        calibration_->laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block & block = raw->blocks[i];
      const uint16_t raw_distance = readLittle16(block.data + k);
      int rotation = block.rotation;
      if (calibration_->num_lasers == 32) {
        const double firing_fraction = config_.model == "32C" ?
          (j / 2) * 2.304 / 55.296 : j * 1.152 / 46.080;
        rotation = (rotation + static_cast<int>(std::lround(azimuth_delta * firing_fraction))) %
          ROTATION_MAX_UNITS;
      }

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        rotation >= config_.min_angle &&
        rotation <= config_.max_angle) ||
        (config_.min_angle > config_.max_angle &&
        (rotation <= config_.max_angle ||
        rotation >= config_.min_angle)))
      {
        float time = time_diff_start_to_this_packet;
        if (!timing_offsets_.empty()) {
          const int timing_block = pkt.data[1204] == 57 ? i / 2 : i;
          time += timing_offsets_[timing_block][j];
        }

        if (raw_distance == 0) {  // no valid laser beam return
          // call to addPoint is still required since output could be organized
          data.addPoint(
            nanf(""), nanf(""), nanf(""), corrections.laser_ring,
            nanf(""), nanf(""), time);
          continue;
        }

        float distance = raw_distance * calibration_->distance_resolution_m;
        distance += corrections.dist_correction;

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
          cos_rot_table_[rotation] * cos_rot_correction +
          sin_rot_table_[rotation] * sin_rot_correction;
        float sin_rot_angle =
          sin_rot_table_[rotation] * cos_rot_correction -
          cos_rot_table_[rotation] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        if (xx < 0.0f) {
          xx = -xx;
        }

        if (yy < 0.0f) {
          yy = -yy;
        }

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;

        if (corrections.two_pt_correction_available) {
          distance_corr_x =
            (corrections.dist_correction - corrections.dist_correction_x) *
            (xx - 2.4f) / (25.04f - 2.4f) +
            corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y =
            (corrections.dist_correction - corrections.dist_correction_y) *
            (yy - 1.93f) / (25.04f - 1.93f) +
            corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
        // the expression wiht '-' is proved to be better than the one with '+'
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */

        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[i].data[k + 2];

        float focal_offset = 256.0f *
          (1.0f - corrections.focal_distance / 13100.0f) *
          (1.0f - corrections.focal_distance / 13100.0f);
        float focal_slope = corrections.focal_slope;
        intensity += focal_slope *
          (std::abs(focal_offset - 256.0f *
            square(1.0f - static_cast<float>(raw_distance) / 65535.0f)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        data.addPoint(
          x_coord, y_coord, z_coord, corrections.laser_ring,
          distance, intensity, time);
      }
    }

    data.newLine();
  }
}

/** @brief convert raw VLS128 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack_vls128(
  const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data,
  const rclcpp::Time & scan_start_time, const raw_packet & decoded)
{
  const raw_packet * raw = &decoded;
  const float time_diff_start_to_this_packet =
    (rclcpp::Time(pkt.stamp) - scan_start_time).seconds();

  const bool dual_return = pkt.data[1204] == 57;
  for (int index = 0; index < BLOCKS_PER_PACKET - (4 * dual_return); ++index) {
    // Dual blocks pair each bank's returns. Group the four banks of each
    // return into a separate organized row so neither return overwrites the other.
    const int block = dual_return ? (index % 4) * 2 + index / 4 : index;
    // cache block for use
    const raw_block & current_block = raw->blocks[block];

    int bank_origin = 0;
    // Used to detect which bank of 32 lasers is in this block
    switch (current_block.header) {
      case VLS128_BANK_1:
        bank_origin = 0;
        break;
      case VLS128_BANK_2:
        bank_origin = 32;
        break;
      case VLS128_BANK_3:
        bank_origin = 64;
        break;
      case VLS128_BANK_4:
        bank_origin = 96;
        break;
      default:
        // ignore packets with mangled or otherwise different contents
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        auto clock = rclcpp::Clock{};
        RCLCPP_WARN_STREAM_THROTTLE(
          rclcpp::get_logger("velodyne_pointcloud"), clock, 60000,
          "skipping invalid VLS-128 packet: block " <<
            block << " header value is " <<
            raw->blocks[block].header);
        return;  // bad packet: skip the rest
    }

    uint16_t azimuth{};        // Uniform initializion. equivalent to azimuth = 0;
    uint16_t azimuth_next{};
    float azimuth_diff{};
    // Calculate difference between current and next block's azimuth angle.
    if (!dual_return) {
      switch (block) {
        // 1 firing sequence span across 4 data block, causing them to have the same azimuth
        case 0:
        case 1:
        case 2:
        case 3:
          azimuth = raw->blocks[0].rotation;
          azimuth_next = raw->blocks[4].rotation;
          break;
        case 4:
        case 5:
        case 6:
        case 7:
        // We assume that the difference between 3rd firing sequence and the next packet firing
        // sequence is the same as difference between 2nd and 3rd firing sequence.
        case 8:
        case 9:
        case 10:
        case 11:
          azimuth = raw->blocks[4].rotation;
          azimuth_next = raw->blocks[8].rotation;
          break;
        default:
          assert(false && "block should be between 0 and BLOCK_PER_PACKET(12)");
          break;
      }
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);
      azimuth = current_block.rotation;
    } else {
      // We can't calculate for dual return, since it have 1 azimuth per data packet and
      // we don't have access to the next packet.
      azimuth = raw->blocks[0].rotation;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      // Offset the laser in this block by which block it's in
      uint8_t laser_number = j + bank_origin;
      // VLS-128 fires 8 lasers at a time
      uint8_t firing_order = laser_number / 8;

      float time {};
      if (timing_offsets_.size()) {
        time = timing_offsets_[dual_return ? 0 : block / 4][firing_order + laser_number / 64] +
          time_diff_start_to_this_packet;
      }

      velodyne_pointcloud::LaserCorrection & corrections =
        calibration_->laser_corrections[laser_number];

      // distance extraction
      const uint16_t raw_distance = readLittle16(current_block.data + k);
      const float distance = raw_distance == 0 ? nanf("") :
        raw_distance * VLS128_DISTANCE_RESOLUTION;

      // correct for the laser rotation as a function of timing during the firings
      float azimuth_corrected_f = azimuth +
        (azimuth_diff * vls_128_laser_azimuth_cache_[firing_order]);
      const uint16_t azimuth_corrected =
        static_cast<int>(std::lround(azimuth_corrected_f)) % ROTATION_MAX_UNITS;

      // condition added to avoid calculating points
      // which are not in the interesting defined area (min_angle < area < max_angle)
      // ((Confusing ! Are min_angle and max_angle included or not ?))
      if (((config_.min_angle < config_.max_angle &&
        azimuth_corrected >= config_.min_angle &&
        azimuth_corrected <= config_.max_angle) ||
        (config_.min_angle > config_.max_angle &&
        (azimuth_corrected <= config_.max_angle ||
        azimuth_corrected >= config_.min_angle))) &&
        pointInRange(distance))
      {
        // convert polar coordinates to Euclidean XYZ
        // Each laser has an vertical offset and
        // an horizontal offset in addition to azimut
        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
          cos_rot_table_[azimuth_corrected] * cos_rot_correction +
          sin_rot_table_[azimuth_corrected] * sin_rot_correction;
        float sin_rot_angle =
          sin_rot_table_[azimuth_corrected] * cos_rot_correction -
          cos_rot_table_[azimuth_corrected] * sin_rot_correction;

        // (TODO) Improve calculation.
        // We can see different calculation in the implementation of other sensors.
        // This implementation is in agreement (if no error was made) with VLS128 manuel,
        // But it might not be the case with physics.

        // The velodyne VLS-128 manual user show this calcultations.
        float x = distance * cos_vert_angle * sin_rot_angle;
        float y = distance * cos_vert_angle * cos_rot_angle;
        float z = distance * sin_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        data.addPoint(
          x_coord,
          y_coord,
          z_coord,
          corrections.laser_ring,
          // azimuth_corrected,
          distance,
          current_block.data[k + 2],  // intensity
          time);
      } else {
        // call to addPoint is still required since output could be organized
        data.addPoint(
          nanf(""), nanf(""), nanf(""), corrections.laser_ring, nanf(""), nanf(""), time);
      }
    }

    if (current_block.header == VLS128_BANK_4) {
      // add a new line only after the last bank (VLS128_BANK_4)
      data.newLine();
    }
  }
}

/** @brief convert raw VLP16 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack_vlp16(
  const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data,
  const rclcpp::Time & scan_start_time, bool is_first_packet, bool is_last_packet,
  const raw_packet & decoded)
{
  float azimuth;
  float azimuth_diff;
  int raw_azimuth_diff;
  float last_azimuth_diff = 0.0f;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  float time_diff_start_to_this_packet =
    (rclcpp::Time(pkt.stamp) - scan_start_time).seconds();

  const raw_packet * raw = &decoded;

  const bool dual_return = pkt.data[1204] == 57;
  if (dual_return && vlp16_packet_timestamp_reference_ >= BLOCKS_PER_PACKET / 2) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("velodyne_pointcloud"),
      "vlp16_packet_timestamp_reference %d is outside the six dual-return block pairs",
      vlp16_packet_timestamp_reference_);
  }
  const bool select_one_return =
    dual_return && vlp16_dual_return_mode_ != Vlp16DualReturnMode::Both;
  const int block_step = select_one_return ? 2 : 1;
  int boundary_block = -1;
  for (int block = 1; block < BLOCKS_PER_PACKET; ++block) {
    if (raw->blocks[block - 1].rotation - raw->blocks[block].rotation > 18000) {
      boundary_block = block;
      break;
    }
  }

  for (int paired_block = 0; paired_block < BLOCKS_PER_PACKET; paired_block += block_step) {
    if (clip_vlp16_scan_boundaries_ && boundary_block >= 0) {
      if (is_first_packet && paired_block < boundary_block) {
        continue;
      }
      if (is_last_packet && paired_block >= boundary_block) {
        continue;
      }
    }
    int block = paired_block;
    // VLP-16 manual, dual-return packet format: each block pair shares one
    // azimuth; the first block holds the last return and the second holds the
    // strongest (or the second strongest when the strongest is also the last).
    if (select_one_return && vlp16_dual_return_mode_ == Vlp16DualReturnMode::Strongest) {
      block += 1;
    }
    const int timing_block = dual_return ? paired_block / 2 : block;

    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header) {
      return;  // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = static_cast<float>(raw->blocks[block].rotation);

    int next_block = block + 1;
    if (dual_return) {
      // Dual-return blocks are adjacent last/strongest pairs with the same
      // azimuth. Interpolate against the next firing-sequence pair.
      next_block = paired_block + 2;
    }
    if (next_block < BLOCKS_PER_PACKET) {
      raw_azimuth_diff = raw->blocks[next_block].rotation - raw->blocks[block].rotation;
      azimuth_diff = static_cast<float>((36000 + raw_azimuth_diff) % 36000);

      // A large negative delta is the normal 360-to-0 scan boundary and the
      // positive modulo above is correct. Reuse the last speed estimate only
      // for a small backwards encoder glitch.
      if (raw_azimuth_diff < 0 && raw_azimuth_diff > -18000) {
        // raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
        // RCLCPP_WARN(
        //   get_logger(), "Packet containing angle overflow, first angle: %u second angle: %u",
        //   raw->blocks[block].rotation, raw->blocks[block+1].rotation);
        // if last_azimuth_diff was not zero, we can assume that the velodyne's
        // speed did not change very much and use the same difference
        if (last_azimuth_diff > 0) {
          azimuth_diff = last_azimuth_diff;
        } else {
          // otherwise we are not able to use this data
          // TODO(somebody): we might just not use the second 16 firings
          continue;
        }
      }

      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
      const int previous_block = block - (dual_return ? 2 : 1);
      if (azimuth_diff <= 0.0f && previous_block >= 0) {
        // Boundary clipping can skip every earlier block of the first packet.
        // Estimate the speed from the preceding block (pair) instead of
        // leaving the firing and per-laser offsets uninterpolated.
        const int previous_diff =
          (36000 + raw->blocks[block].rotation - raw->blocks[previous_block].rotation) % 36000;
        if (previous_diff < 18000) {
          azimuth_diff = static_cast<float>(previous_diff);
        }
      }
    }

    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {
        velodyne_pointcloud::LaserCorrection & corrections = calibration_->laser_corrections[dsr];

        /** Position Calculation */
        const uint16_t raw_distance = readLittle16(raw->blocks[block].data + k);

        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f =
          azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) +
          (firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
        azimuth_corrected = (static_cast<int>(std::round(azimuth_corrected_f)) % 36000);

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((azimuth_corrected >= config_.min_angle &&
          azimuth_corrected <= config_.max_angle &&
          config_.min_angle < config_.max_angle) ||
          (config_.min_angle > config_.max_angle &&
          (azimuth_corrected <= config_.max_angle ||
          azimuth_corrected >= config_.min_angle)))
        {
          // convert polar coordinates to Euclidean XYZ
          float distance = raw_distance == 0 ? nanf("") :
            raw_distance * calibration_->distance_resolution_m;
          distance += corrections.dist_correction;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle =
            cos_rot_table_[azimuth_corrected] * cos_rot_correction +
            sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle =
            sin_rot_table_[azimuth_corrected] * cos_rot_correction -
            cos_rot_table_[azimuth_corrected] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          if (xx < 0.0f) {
            xx = -xx;
          }

          if (yy < 0.0f) {
            yy = -yy;
          }

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;

          if (corrections.two_pt_correction_available) {
            distance_corr_x =
              (corrections.dist_correction - corrections.dist_correction_x) *
              (xx - 2.4f) / (25.04f - 2.4f) + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y =
              (corrections.dist_correction - corrections.dist_correction_y) *
              (yy - 1.93f) / (25.04f - 1.93f) + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

          /** Intensity Calculation */
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[block].data[k + 2];

          float focal_offset = 256.0f * square(1.0f - corrections.focal_distance / 13100.0f);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope *
            (std::abs(focal_offset - 256.0f *
              square(1.0f - static_cast<float>(raw_distance) / 65535.0f)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          float time = 0;
          if (timing_offsets_.size()) {
            time = timing_offsets_[timing_block][firing * 16 + dsr] +
              time_diff_start_to_this_packet -
              timing_offsets_[vlp16_packet_timestamp_reference_][0];
          }

          data.addPoint(
            x_coord, y_coord, z_coord, corrections.laser_ring,
            distance, intensity, time);
        }
      }

      data.newLine();
    }
  }
}
}  // namespace velodyne_rawdata
