 /**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <array>
#include <memory>

namespace eduart {
namespace fleet {

class CollisionAvoidanceLidar : public rclcpp::Node
{
public:
  struct Parameter {
    float distance_reduce_velocity = 0.4f;
    float distance_velocity_zero = 0.05;
  };

  CollisionAvoidanceLidar();
  virtual ~CollisionAvoidanceLidar();

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  enum Area {
    FRONT = 0,
    LEFT,
    RIGHT,
    REAR,
    COUNT
  };

  void callbackLaserScan(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg);
  void callbackVelocity(std::shared_ptr<const geometry_msgs::msg::Twist> msg);

  const Parameter _parameter;
  struct {
    std::array<bool, Area::COUNT> intersection;
    std::array<float, Area::COUNT> reduce_factor;
  } _processing_data;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> _sub_laser_scan;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_velocity;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_velocity;
};

} // end namespace fleet
} // end namespace eduart
