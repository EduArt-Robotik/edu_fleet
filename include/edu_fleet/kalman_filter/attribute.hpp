/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace eduart {
namespace fleet {
namespace kalman_filter {

using Data = double;

namespace impl {

constexpr Data keep_in_range_orientation(const Data value) {
  // keep angle in range of ]-pi, pi]
  Data corrected = value;

  while (corrected >   M_PI) corrected -= 2.0 * M_PI;
  while (corrected <= -M_PI) corrected += 2.0 * M_PI;    

  return corrected;
}

} // end namespace impl

enum class Attribute : std::uint8_t {
  POS_X,
  POS_Y,
  VEL,
  VEL_X,
  VEL_Y,
  ACC,
  ACC_X,
  ACC_Y,
  ROLL,
  PITCH,
  YAW,
  ROLL_RATE,
  PITCH_RATE,
  YAW_RATE,
};

template <Attribute>
struct perform_post_processing {
  inline constexpr Data operator()(const Data value) { return value; }
};

template <>
struct perform_post_processing<Attribute::ROLL> {
  inline constexpr Data operator()(const Data value) {
    // keep angle in range of ]-pi, pi]
    return impl::keep_in_range_orientation(value);
  }
};
template <>
struct perform_post_processing<Attribute::PITCH> {
  inline constexpr Data operator()(const Data value) {
    // keep angle in range of ]-pi, pi]
    return impl::keep_in_range_orientation(value);
  }
};
template <>
struct perform_post_processing<Attribute::YAW> {
  inline constexpr Data operator()(const Data value) {
    // keep angle in range of ]-pi, pi]
    return impl::keep_in_range_orientation(value);
  }
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
