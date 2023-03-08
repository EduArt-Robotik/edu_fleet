/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstddef>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <memory>
#include <vector>

namespace eduart {
namespace fleet {

class FleetControlNode : public rclcpp::Node
{
public:
  struct Parameter {
    std::size_t number_of_robots;
  };

  FleetControlNode();
  ~FleetControlNode() override;

  static Parameter get_parameter(rclcpp::Node& ros_node);

private:
  void callbackTwistFleet(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg);

  Parameter _parameter;

  std::vector<Eigen::MatrixXf> _kinematic_matrix;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> _t_fleet_to_robot;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_twist_fleet;
  std::vector<std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>> _pub_twist_robot;
};

} // end namespace fleet
} // end namespace eduart
