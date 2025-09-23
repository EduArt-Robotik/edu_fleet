 /**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sick_lidar_localization_msgs/msg/odometry_message0104.hpp>

namespace eduart {
namespace fleet {

class SickOdometryRepeater : public rclcpp::Node
{
public:
  SickOdometryRepeater();
  ~SickOdometryRepeater() override = default;

private:
  void callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg);

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odometry;
  std::shared_ptr<rclcpp::Publisher<sick_lidar_localization_msgs::msg::OdometryMessage0104>> _pub_odometry;

  std::uint64_t _telegram_counter = 0;
};

} // end namespace fleet
} // end namespace eduart
