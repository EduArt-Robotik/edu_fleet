#include "edu_fleet/mecanum_path_controller/mecanum_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/time.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace edu_fleet::mecanum_path_controller {

void MecanumController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    [[maybe_unused]] std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
        costmap_ros) {

  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  loadParameters();

  path_handler_ = std::make_unique<edu_fleet::mecanum_path_controller::PathHandler>(
      tf2::durationFromSec(transform_tolerance_), tf_, waypoint_tolerance_);
}

void MecanumController::cleanup() {
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type"
              " edu_fleet::mecanum_path_controller::MecanumController",
              plugin_name_.c_str());
}

void MecanumController::activate() {
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              " edu_fleet::mecanum_path_controller::MecanumController",
              plugin_name_.c_str());
}

void MecanumController::deactivate() {
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              " edu_fleet::mecanum_path_controller::MecanumController",
              plugin_name_.c_str());
}

void MecanumController::setPlan(const nav_msgs::msg::Path &path) {
  path_handler_->setPlan(path);
}

geometry_msgs::msg::TwistStamped MecanumController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    [[maybe_unused]] nav2_core::GoalChecker * /*goal_checker*/) {

  auto local_goal = path_handler_->getLocalGoal(pose);

  auto cmd_vel_raw = computeRawVelocity(pose, local_goal);

  return applyMotionConstraints(cmd_vel_raw, velocity);
}

geometry_msgs::msg::TwistStamped MecanumController::computeRawVelocity(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::PoseStamped &goal_pose) {

  // Position difference in the world frame
  Eigen::Vector3d goal_vector(
      goal_pose.pose.position.x - robot_pose.pose.position.x,
      goal_pose.pose.position.y - robot_pose.pose.position.y,
      goal_pose.pose.position.z - robot_pose.pose.position.z);

  // Goal orientation as an Eigen quaternion
  Eigen::Quaterniond goal_orientation(
      goal_pose.pose.orientation.w, goal_pose.pose.orientation.x,
      goal_pose.pose.orientation.y, goal_pose.pose.orientation.z);
  goal_orientation.normalize();

  // Robot orientation as an Eigen quaternion
  Eigen::Quaterniond robot_orientation(
      robot_pose.pose.orientation.w, robot_pose.pose.orientation.x,
      robot_pose.pose.orientation.y, robot_pose.pose.orientation.z);
  robot_orientation.normalize();

  // Transform position difference into the robot's local frame
  Eigen::Vector3d local_translation =
      robot_orientation.toRotationMatrix().transpose() * goal_vector;

  // Relative orientation: rotation from robot orientation to goal orientation
  Eigen::Quaterniond rotation_difference =
      robot_orientation.conjugate() * goal_orientation;

  rotation_difference.normalize();

  // Convert relative rotation to an angle-axis representation
  Eigen::AngleAxisd angle_axis(rotation_difference);

  // Angular command: relative orientation error
  Eigen::Vector3d rotation_vector = angle_axis.axis() * angle_axis.angle();

  // Construct the TwistStamped command
  geometry_msgs::msg::TwistStamped cmd_vel_raw;
  cmd_vel_raw.header.frame_id = goal_pose.header.frame_id;

  // Linear command: position error in the robot's local frame
  cmd_vel_raw.twist.linear.x = local_translation.x();
  cmd_vel_raw.twist.linear.y = local_translation.y();

  cmd_vel_raw.twist.angular.z = rotation_vector.z();

  return cmd_vel_raw;
}

// TODO: Take deceleration time into account; currently there is no deceleration
// phase
geometry_msgs::msg::TwistStamped MecanumController::applyMotionConstraints(
    const geometry_msgs::msg::TwistStamped &raw_velocity,
    const geometry_msgs::msg::Twist &current_velocity) {
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = raw_velocity.header.frame_id;

  double lower_x_vel =
      std::max(current_velocity.linear.x - (control_duration_ * max_x_accel_),
               -max_x_vel_);
  double upper_x_vel =
      std::min(current_velocity.linear.x + (control_duration_ * max_x_accel_),
               max_x_vel_);

  double lower_y_vel =
      std::max(current_velocity.linear.y - (control_duration_ * max_y_accel_),
               -max_y_vel_);
  double upper_y_vel =
      std::min(current_velocity.linear.y + (control_duration_ * max_y_accel_),
               max_y_vel_);

  double lower_angular_vel = std::max(
      current_velocity.angular.z - (control_duration_ * max_angular_accel_),
      -max_angular_vel_);
  double upper_angular_vel = std::min(
      current_velocity.angular.z + (control_duration_ * max_angular_accel_),
      max_angular_vel_);

  cmd_vel.twist.linear.x =
      std::clamp(raw_velocity.twist.linear.x, lower_x_vel, upper_x_vel);
  cmd_vel.twist.linear.y =
      std::clamp(raw_velocity.twist.linear.y, lower_y_vel, upper_y_vel);
  cmd_vel.twist.angular.z = std::clamp(raw_velocity.twist.angular.z,
                                       lower_angular_vel, upper_angular_vel);

  //   cmd_vel.twist.linear.x = raw_velocity.twist.linear.x;
  //   cmd_vel.twist.linear.y = raw_velocity.twist.linear.y;
  //   cmd_vel.twist.angular.z = raw_velocity.twist.angular.z;

  return cmd_vel;
}

bool MecanumController::cancel() {
  RCLCPP_INFO(logger_, "%s: cancel", __PRETTY_FUNCTION__);
  return true;
}

void MecanumController::reset() {
  RCLCPP_INFO(logger_, "%s: reset", __PRETTY_FUNCTION__);
}

void MecanumController::setSpeedLimit(
    [[maybe_unused]] const double &speed_limit,
    [[maybe_unused]] const bool &percentage) {}

void MecanumController::loadParameters() {
  auto node = node_.lock();

  node->declare_parameter<double>(plugin_name_ + ".max_x_vel", 0.25);
  node->declare_parameter<double>(plugin_name_ + ".max_y_vel", 0.25);
  node->declare_parameter<double>(plugin_name_ + ".max_angular_vel", 1.0);
  node->declare_parameter<double>(plugin_name_ + ".max_x_accel", 0.1);
  node->declare_parameter<double>(plugin_name_ + ".max_y_accel", 0.1);
  node->declare_parameter<double>(plugin_name_ + ".max_angular_accel", 0.4);
  node->declare_parameter<double>(plugin_name_ + ".transform_tolerance", 1.0);
  node->declare_parameter<double>(plugin_name_ + ".waypoint_tolerance", 0.25);

  double control_frequency = 20;
  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  node->get_parameter(plugin_name_ + ".max_x_vel", max_x_vel_);
  node->get_parameter(plugin_name_ + ".max_y_vel", max_y_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_x_accel", max_x_accel_);
  node->get_parameter(plugin_name_ + ".max_y_accel", max_y_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance",
                      transform_tolerance_);
  node->get_parameter(plugin_name_ + ".waypoint_tolerance",
                      waypoint_tolerance_);
}

} // namespace edu_fleet::mecanum_path_controller

PLUGINLIB_EXPORT_CLASS(edu_fleet::mecanum_path_controller::MecanumController,
                       nav2_core::Controller);