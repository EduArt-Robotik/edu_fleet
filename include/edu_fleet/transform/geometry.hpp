/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>

namespace eduart {
namespace fleet {
namespace transform {

void do_transform(
  const std::array<double, 36>& covariance_in, std::array<double, 36>& covariance_out,
  const geometry_msgs::msg::TransformStamped& transform);

void do_transform(
  const std::array<double, 9>& covariance_in, std::array<double, 9>& covariance_out,
  const geometry_msgs::msg::TransformStamped& transform);

void do_transform(
  const geometry_msgs::msg::TwistWithCovariance& twist_in, geometry_msgs::msg::TwistWithCovariance& twist_out,
  const geometry_msgs::msg::TransformStamped& transform);

void do_transform(
  const sensor_msgs::msg::Imu& imu_in, sensor_msgs::msg::Imu& imu_out,
  const geometry_msgs::msg::TransformStamped& transform);

void do_translate(
  const sensor_msgs::msg::Imu& imu_in, sensor_msgs::msg::Imu& imu_out,
  const geometry_msgs::msg::TransformStamped& transform);

void do_rotate(
  const sensor_msgs::msg::Imu& imu_in, sensor_msgs::msg::Imu& imu_out,
  const geometry_msgs::msg::TransformStamped& transform);

} // end namespace transform
} // end namespace fleet
} // end namespace eduart
