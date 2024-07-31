/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/kalman_filter/extended_kalman_filter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include <vector>
#include <cstddef>
#include <memory>

namespace eduart {
namespace fleet {

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
  void callbackPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg, const std::size_t robot_index);

  const Parameter _parameter;

  struct Robot {
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> sub_imu;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> sub_odometry;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> sub_pose;

    std::unique_ptr<kalman_filter::ExtendedKalmanFilter<kalman_filter::FilterModelMecanum::attribute_pack>> kalman_filter;
  };

  std::vector<Robot> _robot;

};

} // end namespace fleet
} // end namespace eduart
