/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstddef>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>

namespace eduart {
namespace fleet {

class TwistAccumulator : public rclcpp::Node
{
public:
  struct Parameter {
    std::size_t num_subscriptions = 2;
  };

  TwistAccumulator();
  ~TwistAccumulator() override;

  static Parameter get_parameter(rclcpp::Node& ros_node);

private:
  void callbackTwistInput(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg, const std::size_t index);

  Parameter _parameter;
  std::vector<geometry_msgs::msg::Twist> _current_input;

  std::vector<std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>>> _sub_twist;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;
};

} // end namespace fleet
} // end namespace eduart
