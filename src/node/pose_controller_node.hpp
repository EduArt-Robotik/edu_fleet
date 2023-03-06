/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <memory>

#include "pid_controller.hpp"

namespace eduart {
namespace fleet {

class PoseController : public rclcpp::Node
{
public:
  struct Parameter {
    PidController::Parameter pid;
  };

  PoseController();
  ~PoseController() override;

  static Parameter get_parameter(rclcpp::Node& ros_node);

private:
  void callbackCurrentPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg);

  Parameter _parameter;
  std::array<PidController, 3> _controller;
  std::unique_ptr<geometry_msgs::msg::Pose> _controller_set_point;
  std::unique_ptr<geometry_msgs::msg::Twist> _controller_output;
  rclcpp::Time _stamp_last_processed;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> _sub_current_pose;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;  
};

} // end namespace fleet
} // end namespace eduart
