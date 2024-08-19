/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/kalman_filter/attribute_pack.hpp>
#include <edu_fleet/kalman_filter/attribute.hpp>

#include <edu_robot/angle.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <stdexcept>

namespace eduart {
namespace fleet {
namespace sensor_model {

using kalman_filter::Data;

template <class = void>
struct message_converting;

template <>
struct message_converting<void> {
  static robot::AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
  }

  static geometry_msgs::msg::Twist to_ros(const Eigen::Vector3d& velocity) {
    geometry_msgs::msg::Twist msg;

    msg.linear.x = velocity.x();
    msg.linear.y = velocity.y();
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = velocity.z();

    return msg;    
  }
  static geometry_msgs::msg::Twist to_ros(const Eigen::Vector2d& linear_velocity, const robot::Angle& angular_velocity) {
    geometry_msgs::msg::Twist msg;

    msg.linear.x = linear_velocity.x();
    msg.linear.y = linear_velocity.y();
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = angular_velocity;

    return msg;    
  } 
};

/**
 * \brief Converts ros messages to measurement vector and covariance matrix.
 */
template <kalman_filter::Attribute... Attributes>
struct message_converting<kalman_filter::AttributePack<Attributes...>> : public message_converting<>
{
  using Pack = kalman_filter::AttributePack<Attributes...>;

  // Helper
  static robot::AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
  }

  // Imu
  static void to_measurement(const sensor_msgs::msg::Imu& imu, Eigen::VectorX<Data>& measurement) {
    using kalman_filter::Attribute;

    if (static_cast<std::size_t>(measurement.size()) != Pack::size()) {
      measurement = Eigen::VectorX<Data>::Zero(Pack::size());
    }

    measurement[Pack::template index<Attribute::ACC_X>()] = imu.linear_acceleration.x;
    measurement[Pack::template index<Attribute::ACC_Y>()] = imu.linear_acceleration.y;

    measurement[Pack::template index<Attribute::YAW_RATE>()] = imu.angular_velocity.z;
  }

  static void to_covariance(const sensor_msgs::msg::Imu& imu, Eigen::MatrixX<Data>& covariance) {
    using kalman_filter::Attribute;

    if (static_cast<std::size_t>(covariance.rows()) != Pack::size() ||
        static_cast<std::size_t>(covariance.cols()) != Pack::size()) {
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

    // angular velocity
    covariance(YAW_RATE, YAW_RATE) = imu.angular_velocity_covariance[8];
  }

  // Odometry
  static void to_measurement(const nav_msgs::msg::Odometry& odometry, Eigen::VectorX<Data>& measurement) {
    using kalman_filter::Attribute;

    if (static_cast<std::size_t>(measurement.size()) != Pack::size()) {
      measurement = Eigen::VectorX<Data>::Zero(Pack::size());
    }

    // linear velocity
    measurement[Pack::template index<Attribute::VEL_X>()] = odometry.twist.twist.linear.x;
    measurement[Pack::template index<Attribute::VEL_Y>()] = odometry.twist.twist.linear.y;

    // angular velocity
    measurement[Pack::template index<Attribute::YAW_RATE>()] = odometry.twist.twist.angular.z;
  }

  static void to_covariance(const nav_msgs::msg::Odometry& odometry, Eigen::MatrixX<Data>& covariance) {
    using kalman_filter::Attribute;

    if (static_cast<std::size_t>(covariance.rows()) != Pack::size() ||
        static_cast<std::size_t>(covariance.cols()) != Pack::size()) {
      covariance = Eigen::MatrixX<Data>::Identity(Pack::size(), Pack::size()) * static_cast<Data>(1000);
    }

    constexpr std::size_t VEL_X = Pack::template index<Attribute::VEL_X>();
    constexpr std::size_t VEL_Y = Pack::template index<Attribute::VEL_Y>();
    constexpr std::size_t YAW_RATE = Pack::template index<Attribute::YAW_RATE>();

    // linear velocity
    covariance(VEL_X, VEL_X) = odometry.twist.covariance[0];
    covariance(VEL_X, VEL_Y) = odometry.twist.covariance[1];
    covariance(VEL_Y, VEL_X) = odometry.twist.covariance[6];
    covariance(VEL_Y, VEL_Y) = odometry.twist.covariance[7];

    // angular velocity
    covariance(YAW_RATE, YAW_RATE) = odometry.twist.covariance[35];
  }

  // Pose
  static void to_measurement(const geometry_msgs::msg::PoseWithCovarianceStamped& pose, Eigen::VectorX<Data>& measurement) {
    using kalman_filter::Attribute;

    if (static_cast<std::size_t>(measurement.size()) != Pack::size()) {
      measurement = Eigen::VectorX<Data>::Zero(Pack::size());
    }

    // position
    measurement[Pack::template index<Attribute::W_POS_X>()] = pose.pose.pose.position.x;
    measurement[Pack::template index<Attribute::W_POS_Y>()] = pose.pose.pose.position.y;

    // orientation
    measurement[Pack::template index<Attribute::W_YAW>()] = quaternion_to_yaw(pose.pose.pose.orientation);
  }

  static void to_covariance(const geometry_msgs::msg::PoseWithCovarianceStamped& pose, Eigen::MatrixX<Data>& covariance) {
    using kalman_filter::Attribute;

    if (static_cast<std::size_t>(covariance.rows()) != Pack::size() ||
        static_cast<std::size_t>(covariance.cols()) != Pack::size()) {
      covariance = Eigen::MatrixX<Data>::Identity(Pack::size(), Pack::size()) * static_cast<Data>(1000);
    }

    constexpr std::size_t W_POS_X = Pack::template index<Attribute::W_POS_X>();
    constexpr std::size_t W_POS_Y = Pack::template index<Attribute::W_POS_Y>();
    constexpr std::size_t W_YAW = Pack::template index<Attribute::W_YAW>();

    // position
    covariance(W_POS_X, W_POS_X) = pose.pose.covariance[0];
    covariance(W_POS_X, W_POS_Y) = pose.pose.covariance[1];
    covariance(W_POS_Y, W_POS_X) = pose.pose.covariance[6];
    covariance(W_POS_Y, W_POS_Y) = pose.pose.covariance[7];

    // orientation
    covariance(W_YAW, W_YAW) = pose.pose.covariance[35];
  }


  // to ros messages
  static nav_msgs::msg::Odometry to_ros(const Eigen::VectorX<Data>& state, const Eigen::MatrixX<Data>& covariance) {
    using kalman_filter::Attribute;

    if (static_cast<std::size_t>(state.size()) != Pack::size()) {
      throw std::invalid_argument("ro_ros(): attributes doesn't fit to given state vector.");
    }    
    if (static_cast<std::size_t>(covariance.rows()) != Pack::size() ||
        static_cast<std::size_t>(covariance.cols()) != Pack::size()) {
      throw std::invalid_argument("ro_ros(): attributes doesn't fit to given covariance matrix.");
    }

    constexpr std::size_t W_POS_X = Pack::template index<Attribute::W_POS_X>();
    constexpr std::size_t W_POS_Y = Pack::template index<Attribute::W_POS_Y>();
    constexpr std::size_t W_YAW = Pack::template index<Attribute::W_YAW>();    

    constexpr std::size_t VEL_X = Pack::template index<Attribute::VEL_X>();
    constexpr std::size_t VEL_Y = Pack::template index<Attribute::VEL_Y>();
    constexpr std::size_t YAW_RATE = Pack::template index<Attribute::YAW_RATE>();

    // converting data
    nav_msgs::msg::Odometry msg;

    // pose
    msg.pose.pose.position.x = state[W_POS_X];
    msg.pose.pose.position.y = state[W_POS_Y];

    const Eigen::Quaterniond q(Eigen::AngleAxisd(state[W_YAW], Eigen::Vector3d::UnitZ()));
    msg.pose.pose.orientation.w = q.w();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    
    msg.pose.covariance[ 0] = covariance(W_POS_X, W_POS_X);
    msg.pose.covariance[ 7] = covariance(W_POS_Y, W_POS_Y);
    msg.pose.covariance[35] = covariance(W_YAW, W_YAW);

    // twist
    msg.twist.twist.linear.x  = state[VEL_X];
    msg.twist.twist.linear.y  = state[VEL_Y];
    msg.twist.twist.angular.z = state[YAW_RATE];

    msg.twist.covariance[ 0] = covariance(VEL_X, VEL_X);
    msg.twist.covariance[ 7] = covariance(VEL_Y, VEL_Y);
    msg.twist.covariance[35] = covariance(YAW_RATE, YAW_RATE);

    return msg;
  }
};

} // end namespace sensor_model
} // end namespace fleet
} // end namespace eduart
