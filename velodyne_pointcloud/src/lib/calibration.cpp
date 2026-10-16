// Copyright 2012, 2019 Austin Robot Technology, Piyush Khandelwal, Joshua Whitley
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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "velodyne_pointcloud/calibration.hpp"

namespace YAML
{

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator>>(const YAML::Node & node, T & i)
{
  i = node.as<T>();
}

}  // namespace YAML

namespace velodyne_pointcloud
{

constexpr char NUM_LASERS[] = "num_lasers";
constexpr char DISTANCE_RESOLUTION[] = "distance_resolution";
constexpr char LASERS[] = "lasers";
constexpr char LASER_ID[] = "laser_id";
constexpr char ROT_CORRECTION[] = "rot_correction";
constexpr char VERT_CORRECTION[] = "vert_correction";
constexpr char DIST_CORRECTION[] = "dist_correction";
constexpr char TWO_PT_CORRECTION_AVAILABLE[] =
  "two_pt_correction_available";
constexpr char DIST_CORRECTION_X[] = "dist_correction_x";
constexpr char DIST_CORRECTION_Y[] = "dist_correction_y";
constexpr char VERT_OFFSET_CORRECTION[] = "vert_offset_correction";
constexpr char HORIZ_OFFSET_CORRECTION[] = "horiz_offset_correction";
constexpr char MAX_INTENSITY[] = "max_intensity";
constexpr char MIN_INTENSITY[] = "min_intensity";
constexpr char FOCAL_DISTANCE[] = "focal_distance";
constexpr char FOCAL_SLOPE[] = "focal_slope";
constexpr char ENCODER_ECCENTRICITY_MODEL[] = "encoder_eccentricity_model";

/** Read calibration for a single laser. */
void operator>>(const YAML::Node & node, std::pair<int, LaserCorrection> & correction)
{
  node[LASER_ID] >> correction.first;
  node[ROT_CORRECTION] >> correction.second.rot_correction;
  node[VERT_CORRECTION] >> correction.second.vert_correction;
  node[DIST_CORRECTION] >> correction.second.dist_correction;

  if (node[TWO_PT_CORRECTION_AVAILABLE]) {
    node[TWO_PT_CORRECTION_AVAILABLE] >>
    correction.second.two_pt_correction_available;
  } else {
    correction.second.two_pt_correction_available = false;
  }

  node[DIST_CORRECTION_X] >> correction.second.dist_correction_x;
  node[DIST_CORRECTION_Y] >> correction.second.dist_correction_y;
  node[VERT_OFFSET_CORRECTION] >> correction.second.vert_offset_correction;

  if (node[HORIZ_OFFSET_CORRECTION]) {
    node[HORIZ_OFFSET_CORRECTION] >>
    correction.second.horiz_offset_correction;
  } else {
    correction.second.horiz_offset_correction = 0;
  }

  float max_intensity_float = 255.0;

  if (node[MAX_INTENSITY]) {
    node[MAX_INTENSITY] >> max_intensity_float;
  }


  float min_intensity_float = 0.0;

  if (node[MIN_INTENSITY]) {
    node[MIN_INTENSITY] >> min_intensity_float;
  }

  if (!std::isfinite(min_intensity_float) || !std::isfinite(max_intensity_float) ||
    min_intensity_float < 0.0f || max_intensity_float > 255.0f ||
    min_intensity_float > max_intensity_float)
  {
    throw std::runtime_error("Invalid calibration intensity limits");
  }
  correction.second.min_intensity = ::floorf(min_intensity_float);
  correction.second.max_intensity = ::floorf(max_intensity_float);

  node[FOCAL_DISTANCE] >> correction.second.focal_distance;
  node[FOCAL_SLOPE] >> correction.second.focal_slope;

  // Calculate cached values
  correction.second.cos_rot_correction =
    ::cosf(correction.second.rot_correction);
  correction.second.sin_rot_correction =
    ::sinf(correction.second.rot_correction);
  correction.second.cos_vert_correction =
    ::cosf(correction.second.vert_correction);
  correction.second.sin_vert_correction =
    ::sinf(correction.second.vert_correction);

  correction.second.laser_ring = 0;   // clear initially (set later)
}

/** Read entire calibration file. */
void operator>>(const YAML::Node & node, Calibration & calibration)
{
  int num_lasers;
  node[NUM_LASERS] >> num_lasers;
  float distance_resolution_m;
  node[DISTANCE_RESOLUTION] >> distance_resolution_m;
  const YAML::Node & lasers = node[LASERS];
  if (num_lasers <= 0 || num_lasers > 128 || !lasers.IsSequence() ||
    lasers.size() != static_cast<size_t>(num_lasers) ||
    !std::isfinite(distance_resolution_m) || distance_resolution_m <= 0.0f)
  {
    throw std::runtime_error("Invalid calibration laser count or distance resolution");
  }
  calibration.laser_corrections.clear();
  calibration.num_lasers = num_lasers;
  calibration.distance_resolution_m = distance_resolution_m;
  calibration.has_encoder_eccentricity_model = false;
  calibration.encoder_eccentricity_model = {0.0, 0.0};
  if (node[ENCODER_ECCENTRICITY_MODEL]) {
    const YAML::Node model = node[ENCODER_ECCENTRICITY_MODEL];
    if (!model.IsSequence() || model.size() != 2) {
      throw std::runtime_error("encoder_eccentricity_model must contain two values");
    }
    calibration.encoder_eccentricity_model = {
      model[0].as<double>(), model[1].as<double>()};
    if (!std::isfinite(calibration.encoder_eccentricity_model[0]) ||
      !std::isfinite(calibration.encoder_eccentricity_model[1]))
    {
      throw std::runtime_error("encoder_eccentricity_model values must be finite");
    }
    // Physical encoder eccentricities are a few thousandths of the encoder
    // radius. The Newton inverse below stops converging near 0.9.
    if (!(std::hypot(
        calibration.encoder_eccentricity_model[0],
        calibration.encoder_eccentricity_model[1]) < 0.1))
    {
      throw std::runtime_error("encoder_eccentricity_model magnitude must be below 0.1");
    }
    calibration.has_encoder_eccentricity_model = true;
  }
  calibration.laser_corrections.resize(num_lasers);

  std::vector<bool> seen(num_lasers, false);
  for (int i = 0; i < num_lasers; i++) {
    std::pair<int, LaserCorrection> correction;
    lasers[i] >> correction;
    const int index = correction.first;

    if (index < 0 || index >= num_lasers || seen[index]) {
      throw std::runtime_error("Calibration laser IDs must be unique and in range");
    }
    const auto & laser = correction.second;
    for (const float value : {laser.rot_correction, laser.vert_correction,
        laser.dist_correction, laser.dist_correction_x, laser.dist_correction_y,
        laser.vert_offset_correction, laser.horiz_offset_correction,
        laser.focal_distance, laser.focal_slope})
    {
      if (!std::isfinite(value)) {
        throw std::runtime_error("Calibration corrections must be finite");
      }
    }
    seen[index] = true;

    calibration.laser_corrections[index] = (correction.second);
  }

  // Stable sorting gives every laser a unique ring, including equal angles.
  std::vector<int> indices(num_lasers);
  std::iota(indices.begin(), indices.end(), 0);
  std::stable_sort(indices.begin(), indices.end(), [&calibration](int left, int right) {
      return calibration.laser_corrections[left].vert_correction <
             calibration.laser_corrections[right].vert_correction;
    });
  for (int ring = 0; ring < num_lasers; ++ring) {
    calibration.laser_corrections[indices[ring]].laser_ring = ring;
  }
}

float Calibration::correctedAzimuth(float measured_angle_rad) const
{
  if (!has_encoder_eccentricity_model) {
    return measured_angle_rad;
  }

  // Exyn's archived calibration implementation defines the forward model as
  // measured = atan2(sin(true) + cy, cos(true) + cx).  The mapping is smooth
  // and monotonic for the small physical eccentricities, so Newton iteration
  // gives a deterministic inverse without storing a 36,000-entry YAML LUT.
  const double cx = encoder_eccentricity_model[0];
  const double cy = encoder_eccentricity_model[1];
  const double measured = measured_angle_rad;
  double angle = measured;
  for (int iteration = 0; iteration < 6; ++iteration) {
    const double sine = std::sin(angle);
    const double cosine = std::cos(angle);
    const double x = cosine + cx;
    const double y = sine + cy;
    const double predicted = std::atan2(y, x);
    const double residual = std::atan2(
      std::sin(predicted - measured), std::cos(predicted - measured));
    const double derivative = (x * cosine + y * sine) / (x * x + y * y);
    angle -= residual / derivative;
  }
  const double predicted = std::atan2(std::sin(angle) + cy, std::cos(angle) + cx);
  const double residual = std::atan2(
    std::sin(predicted - measured), std::cos(predicted - measured));
  if (!(std::abs(residual) < 1e-9)) {
    throw std::runtime_error("encoder eccentricity inverse did not converge");
  }
  return static_cast<float>(angle);
}

Calibration::Calibration(const std::string & calibration_file)
: distance_resolution_m(0.002f)
{
  std::ifstream fin(calibration_file.c_str());

  if (!fin.is_open()) {
    throw std::runtime_error("Failed to open calibration file");
  }

  try {
    YAML::Node doc;

    fin.close();
    doc = YAML::LoadFile(calibration_file);

    doc >> *this;
  } catch (YAML::Exception & e) {
    fin.close();
    throw std::runtime_error("YAML Exception: " + std::string(e.what()));
  }

  fin.close();
}

}  // namespace velodyne_pointcloud
