/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/kalman_filter/attribute_pack.hpp>
#include <edu_fleet/kalman_filter/attribute.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Core>

namespace eduart {
namespace fleet {
namespace sensor_model {

using kalman_filter::Data;

template <class>
struct message_converting;

template <kalman_filter::Attribute... Attributes>
struct message_converting<kalman_filter::AttributePack<Attributes...>> {
  using Pack = kalman_filter::AttributePack<Attributes...>;

  static void to_measurement(const sensor_msgs::msg::Imu& imu, Eigen::VectorX<Data>& measurement) {
    using kalman_filter::Attribute;

    if (measurement.size() != Pack::size()) {
      measurement = Eigen::VectorX<Data>::Zero(Pack::size());
    }

    measurement[Pack::template index<Attribute::ACC_X>()] = imu.linear_acceleration.x;
    measurement[Pack::template index<Attribute::ACC_Y>()] = imu.linear_acceleration.y;

    measurement[Pack::template index<Attribute::YAW_RATE>()] = imu.angular_velocity.z;
  }

  static void to_covariance(const sensor_msgs::msg::Imu& imu, Eigen::MatrixX<Data>& covariance) {
    using kalman_filter::Attribute;

    if (covariance.rows() != Pack::size() || covariance.cols() != Pack::size()) {
      covariance = Eigen::MatrixX<Data>::Identity(Pack::size(), Pack::size()) * static_cast<Data>(1000);
    }

    constexpr std::size_t ACC_X = Pack::template index<Attribute::ACC_X>();
    constexpr std::size_t ACC_Y = Pack::template index<Attribute::ACC_Y>();
    constexpr std::size_t YAW_RATE = Pack::template index<Attribute::YAW_RATE>();

    // linear acceleration
    covariance(ACC_X, ACC_X) = imu.linear_acceleration_covariance[0];
    covariance(ACC_X, ACC_Y) = imu.linear_acceleration_covariance[1];
    covariance(ACC_Y, ACC_X) = imu.linear_acceleration_covariance[3];
    covariance(ACC_Y, ACC_Y) = imu.linear_acceleration_covariance[4];
    covariance(ACC_X, ACC_X) = imu.linear_acceleration_covariance[0];

    // angular velocity
    covariance(YAW_RATE, YAW_RATE) = imu.angular_velocity_covariance[8];
  }
};

} // end namespace sensor_model
} // end namespace fleet
} // end namespace eduart