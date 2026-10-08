 /**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace eduart {
namespace fleet {

class TritonPoseRepeater : public rclcpp_lifecycle::LifecycleNode
{
public:
  struct Parameter {

  };

  TritonPoseRepeater();
  ~TritonPoseRepeater() override = default;

  static Parameter get_parameter(const Parameter &default_parameter, rclcpp_lifecycle::LifecycleNode &ros_node);

private:
  void callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg);

  const Parameter _parameter;

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odometry;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> _pub_pose;
};

} // end namespace fleet
} // end namespace eduart
