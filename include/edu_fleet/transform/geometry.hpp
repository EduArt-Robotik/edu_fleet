/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <array>
#include <geometry_msgs/msg/detail/twist_with_covariance__struct.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace eduart {
namespace fleet {
namespace transform {

void do_transform(
  const std::array<double, 36>& covariance_in, std::array<double, 36>& covariance_out,
  const geometry_msgs::msg::TransformStamped& transform);

void do_transform(
  const geometry_msgs::msg::TwistWithCovariance& twist_in, geometry_msgs::msg::TwistWithCovariance& twist_out,
  const geometry_msgs::msg::TransformStamped& transform);

} // end namespace transform
} // end namespace fleet
} // end namespace eduart
