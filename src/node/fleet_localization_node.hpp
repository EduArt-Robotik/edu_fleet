/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_pack.hpp>
#include <edu_fleet/kalman_filter/extended_kalman_filter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>
#include <edu_fleet/sensor_model/sensor_model_ros.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include <vector>
#include <cstddef>
#include <memory>

namespace eduart {
namespace fleet {

using kalman_filter::FilterModelMecanum;
using kalman_filter::ExtendedKalmanFilter;
using kalman_filter::Attribute;
using kalman_filter::AttributePack;
using sensor_model::SensorModelRos;

class FleetLocalization : public rclcpp::Node
{
public:
  struct Parameter {
    std::vector<std::string> robot_name;
    std::vector<kalman_filter::FilterModelMecanum::Parameter> filter_parameter;

    inline std::size_t number_of_robots() const { return robot_name.size(); }
  };

  FleetLocalization(const Parameter& parameter);
  ~FleetLocalization() override;

  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void callbackImu(std::shared_ptr<const sensor_msgs::msg::Imu> msg, const std::size_t robot_index);
  void callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg, const std::size_t robot_index);
  void callbackPose(std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg, const std::size_t robot_index);
  
  const Parameter _parameter;

  struct Robot {
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> sub_imu;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> sub_odometry;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> sub_pose;

    std::unique_ptr<ExtendedKalmanFilter<FilterModelMecanum::attribute_pack>> kalman_filter;
  };

  std::vector<Robot> _robot;

  // sensor models
  using SensorModelImu = SensorModelRos<AttributePack<Attribute::ACC_X, Attribute::ACC_Y, Attribute::YAW_RATE>, sensor_msgs::msg::Imu>;
  using SensorModelOdometry = SensorModelRos<AttributePack<Attribute::VEL_X, Attribute::VEL_Y, Attribute::YAW_RATE>, nav_msgs::msg::Odometry>;
  using SensorModelPose = SensorModelRos<AttributePack<Attribute::W_POS_X, Attribute::W_POS_Y, Attribute::W_YAW>, geometry_msgs::msg::PoseWithCovarianceStamped>;

  std::shared_ptr<SensorModelImu> _sensor_model_imu;
  std::shared_ptr<SensorModelOdometry> _sensor_model_odometry;
  std::shared_ptr<SensorModelPose> _sensor_model_pose;
};

} // end namespace fleet
} // end namespace eduart
