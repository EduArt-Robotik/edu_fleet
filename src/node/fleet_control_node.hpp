/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/srv/get_transform.hpp>

#include <edu_robot/rpm.hpp>
#include <edu_robot/angle.hpp>

#include <edu_robot/msg/robot_kinematic_description.hpp>
#include <edu_robot/msg/set_lighting_color.hpp>
#include <edu_robot/msg/robot_status_report.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <chrono>

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
    std::chrono::milliseconds process_interval{10}; // 10ms/100Hz
    std::chrono::milliseconds timeout{250}; // 250ms
    double drift_limit = 0.1; // 0.1m
    std::string world_frame_id = "map";
  };

  FleetControlNode();
  ~FleetControlNode() override;

  static Parameter get_parameter(rclcpp::Node& ros_node);

private:
  void process();
  void callbackTwistFleet(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg);
  void callbackServiceGetTransform(
    const std::shared_ptr<edu_fleet::srv::GetTransform::Request> request,
    std::shared_ptr<edu_fleet::srv::GetTransform::Response> response);
  void callbackRobotStatusReport(
    std::shared_ptr<const edu_robot::msg::RobotStatusReport> report, const std::size_t robot_index);
  void callbackLocalization(
    std::shared_ptr<const nav_msgs::msg::Odometry> msg, const std::size_t robot_index);
  rcl_interfaces::msg::SetParametersResult callbackParameter(const std::vector<rclcpp::Parameter>& parameters);
  void processKinematicDescription(
    std::shared_ptr<const edu_robot::msg::RobotKinematicDescription> description, const std::size_t robot_index);

  visualization_msgs::msg::MarkerArray getDebugMessage(const rclcpp::Time stamp) const;
  nav_msgs::msg::Odometry getOdometryMessage(const rclcpp::Time stamp) const;

  Parameter _parameter;
  Eigen::Vector2d _fleet_position; //> not used for the moment, or not really
  robot::AnglePiToPi _fleet_orientation;
  rclcpp::Time _stamp_last_processing;
  rclcpp::Time _stamp_last_twist_received;
  double _velocity_reduce_factor = 1.0; //> used to reduce robots velocity to keep motors in physical limits
  Eigen::Vector3d _fleet_velocity;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_twist_fleet;
  std::shared_ptr<rclcpp::Service<edu_fleet::srv::GetTransform>> _srv_server_get_transform;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _pub_fleet_odometry;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> _pub_visualization; //> used to debug target pose
  std::shared_ptr<rclcpp::TimerBase> _timer_processing;
  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> _parameter_handle;

  struct Robot {
    // robot related things
    Eigen::MatrixXd kinematic_matrix;
    Eigen::Matrix3d t_fleet_to_robot_velocity;
    Eigen::Matrix3d t_fleet_to_robot_transform;
    std::vector<eduart::robot::Rpm> rpm_limit;
    std::uint8_t lost_fleet_formation; //> value != 0 indicates fleet formation is lost
    std::uint8_t current_mode;

    Eigen::Vector3d velocity;
    Eigen::Vector2d position;
    robot::AnglePiToPi orientation;
    Eigen::Vector2d target_position;
    robot::AnglePiToPi target_orientation;

    // subscriptions
    std::shared_ptr<rclcpp::Subscription<edu_robot::msg::RobotStatusReport>> sub_robot_status;
    std::shared_ptr<rclcpp::Subscription<edu_robot::msg::RobotKinematicDescription>> sub_kinematic_description;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> sub_localization;

    // publisher
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> pub_twist_robot;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pub_target_pose;
    std::shared_ptr<rclcpp::Publisher<edu_robot::msg::SetLightingColor>> pub_set_lighting;
  };

  std::vector<Robot> _robot;
};

} // end namespace fleet
} // end namespace eduart
