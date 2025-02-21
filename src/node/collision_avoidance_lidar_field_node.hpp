/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <edu_perception/msg/lidar_field_evaluation.hpp>

namespace eduart {
namespace fleet {

class CollisionAvoidanceLidarField : public rclcpp::Node
{
public:
  struct Parameter {
    double velocity_warnfield_linear = 0.1;
    double velocity_warnfield_angular = 0.1;
  };

  CollisionAvoidanceLidarField();
  ~CollisionAvoidanceLidarField() override;

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void callbackTwist(std::shared_ptr<const geometry_msgs::msg::Twist> msg);
  void callbackLidarField(std::shared_ptr<const edu_perception::msg::LidarFieldEvaluation> msg);

  const Parameter _parameter;
  bool _warnfield_active = false;
  bool _safetyfield_active = false;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_twist;
  std::shared_ptr<rclcpp::Subscription<edu_perception::msg::LidarFieldEvaluation>> _sub_field;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;
};

} // end namespace fleet
} // end namespace eduart
