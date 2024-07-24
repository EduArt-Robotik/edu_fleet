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
  W_POS_X,      //> position x in world coordinate system
  W_POS_Y,      //> position y in world coordinate system
  VEL,          //> absolute velocity in vehicle coordinate system
  VEL_X,        //> velocity x in vehicle coordinate system
  VEL_Y,        //> velocity y in vehicle coordinate system
  ACC,          //> absolute acceleration in vehicle coordinate system
  ACC_X,        //> acceleration x in vehicle coordinate system
  ACC_Y,        //> acceleration y in vehicle coordinate system
  W_ROLL,       //> roll angle in world coordinate system
  W_PITCH,      //> pitch angel in world coordinate system
  W_YAW,        //> yaw angle in world coordinate system
  ROLL_RATE,    //> roll rate in vehicle coordinate system
  PITCH_RATE,   //> pitch rate in vehicle coordinate system
  YAW_RATE,     //> yaw rate in vehicle coordinate system
};

template <Attribute>
struct perform_post_processing {
  inline constexpr Data operator()(const Data value) { return value; }
};

template <>
struct perform_post_processing<Attribute::W_ROLL> {
  inline constexpr Data operator()(const Data value) {
    // keep angle in range of ]-pi, pi]
    return impl::keep_in_range_orientation(value);
  }
};
template <>
struct perform_post_processing<Attribute::W_PITCH> {
  inline constexpr Data operator()(const Data value) {
    // keep angle in range of ]-pi, pi]
    return impl::keep_in_range_orientation(value);
  }
};
template <>
struct perform_post_processing<Attribute::W_YAW> {
  inline constexpr Data operator()(const Data value) {
    // keep angle in range of ]-pi, pi]
    return impl::keep_in_range_orientation(value);
  }
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
