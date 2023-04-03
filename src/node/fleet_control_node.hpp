/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_swarm/srv/get_transform.hpp>

#include <edu_robot/srv/get_kinematic_description.hpp>
#include <edu_robot/rotation_per_minute.hpp>

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

    struct Pose2D {
      double x = 0.0;
      double y = 0.0;
      double yaw = 0.0;
    };
    std::vector<Pose2D> robot_pose;
    std::vector<std::string> robot_name;
  };

  FleetControlNode();
  ~FleetControlNode() override;

  static Parameter get_parameter(rclcpp::Node& ros_node);

private:
  void callbackTwistFleet(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg);
  void callbackServiceGetTransform(
    const std::shared_ptr<edu_swarm::srv::GetTransform::Request> request,
    std::shared_ptr<edu_swarm::srv::GetTransform::Response> response);
  // void updateKinematicDescription();
  void processKinematicDescription(
    std::shared_ptr<const edu_robot::msg::RobotKinematicDescription> description, const std::size_t robot_index);

  Parameter _parameter;

  std::vector<Eigen::MatrixXd> _kinematic_matrix;
  std::vector<std::vector<eduart::robot::Rpm>> _robot_rpm_limit;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> _t_fleet_to_robot;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_twist_fleet;
  std::vector<std::shared_ptr<rclcpp::Subscription<edu_robot::msg::RobotKinematicDescription>>> _sub_kinematic_description;
  std::vector<std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>> _pub_twist_robot;
  // std::vector<std::shared_ptr<rclcpp::Client<edu_robot::srv::GetKinematicDescription>>> _srv_client_get_kinematic;
  std::shared_ptr<rclcpp::Service<edu_swarm::srv::GetTransform>> _srv_server_get_transform;
  // std::shared_ptr<rclcpp::TimerBase> _timer_update_kinematic;
};

} // end namespace fleet
} // end namespace eduart
