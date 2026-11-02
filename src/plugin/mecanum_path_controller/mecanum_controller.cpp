#include "edu_fleet/mecanum_path_controller/mecanum_controller.hpp"
#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <angles/angles.h>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/time.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/utils.hpp>

namespace edu_fleet::mecanum_path_controller {

void
MecanumController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  [[maybe_unused]] std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
    costmap_ros
)
{

  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  loadParameters();

  path_handler_ =
    std::make_unique<edu_fleet::mecanum_path_controller::PathHandler>(
      tf2::durationFromSec(transform_tolerance_), tf_, waypoint_tolerance_
    );

  local_goal_publisher_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "local_goal",
      rclcpp::QoS(1)
    );

  corner_lookahead_ = 2;
  corner_lookback_ = 3;
  apply_corner_velocity_scaling_ = true;
  min_corner_scaling_factor_ = 0.15;
  start_corner_scaling_angle_ = 0.3;
  end_corner_scaling_angle_ = 1.5;

  linear_gain_ = 2.0;
  angular_gain_ = 2.0;
}

void
MecanumController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s of type"
                       " edu_fleet::mecanum_path_controller::MecanumController",
              plugin_name_.c_str());
  local_goal_publisher_.reset();
}

void
MecanumController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller: %s of type "
                       " edu_fleet::mecanum_path_controller::MecanumController",
              plugin_name_.c_str());
  local_goal_publisher_->on_activate();
}

void
MecanumController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller: %s of type "
                       " edu_fleet::mecanum_path_controller::MecanumController",
              plugin_name_.c_str());
  local_goal_publisher_->on_deactivate();
}

void
MecanumController::setPlan(const nav_msgs::msg::Path& path)
{
  path_handler_->setPlan(path);
}

geometry_msgs::msg::TwistStamped
MecanumController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& velocity,
  [[maybe_unused]] nav2_core::GoalChecker* /*goal_checker*/
)
{
  auto error = calculateError(pose);
  auto raw_twist = calculateRawTwist(error, pose.header.frame_id);

  // TODO: add methods to calculate scaling factors like corner scaling/distance to goal with unifies signature and execute all methods to compute the lowest factor and apply it


  return raw_twist;


  auto local_goal = path_handler_->getLocalGoal(pose);

  if (local_goal_publisher_ && local_goal_publisher_->is_activated()) {
    local_goal_publisher_->publish(local_goal);
  }

  auto cmd_vel_raw = computeRawVelocity(pose, local_goal);

  if (apply_corner_velocity_scaling_ == true)
    applyCornerSpeedScaling(cmd_vel_raw);

  auto motion = applyMotionConstraints(cmd_vel_raw, velocity, computeMaxStoppingVelocity(pose));

  return motion;
}

geometry_msgs::msg::TwistStamped
MecanumController::computeRawVelocity(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::PoseStamped& goal_pose
)
{

  // Position difference in the world frame
  Eigen::Vector3d goal_vector(
    goal_pose.pose.position.x - robot_pose.pose.position.x,
    goal_pose.pose.position.y - robot_pose.pose.position.y,
    goal_pose.pose.position.z - robot_pose.pose.position.z
  );

  // Goal orientation as an Eigen quaternion
  Eigen::Quaterniond goal_orientation(
    goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z
  );
  goal_orientation.normalize();

  // Robot orientation as an Eigen quaternion
  Eigen::Quaterniond robot_orientation(
    robot_pose.pose.orientation.w, robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z
  );
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
  // cmd_vel_raw.twist.angular.z = 0.0;

  return cmd_vel_raw;
}

void
MecanumController::applyCornerSpeedScaling(geometry_msgs::msg::TwistStamped& raw_velocity)
{
  const double max_angle = path_handler_->getMaxAngle(corner_lookahead_, corner_lookback_);
  double scaling_factor;

  if (max_angle < start_corner_scaling_angle_) {
    // skip for straight ish paths
    RCLCPP_INFO(logger_, "corner angle (%f) blow threshold -> no scaling", max_angle);
    return;
  }
  if (max_angle > end_corner_scaling_angle_) // stop at max value to prevent negative scaling etc.
    scaling_factor = min_corner_scaling_factor_;
  else {
    scaling_factor = 1 + (((min_corner_scaling_factor_ - 1) / (end_corner_scaling_angle_ - start_corner_scaling_angle_)) * (max_angle - start_corner_scaling_angle_));
  }

  raw_velocity.twist.linear.x *= scaling_factor;
  raw_velocity.twist.linear.y *= scaling_factor;
}

geometry_msgs::msg::TwistStamped
MecanumController::applyMotionConstraints(
  const geometry_msgs::msg::TwistStamped& raw_velocity,
  const geometry_msgs::msg::Twist& current_velocity,
  const geometry_msgs::msg::Twist& max_stopping_velocity
)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = raw_velocity.header;

  // Calculate translational constraints
  const double lower_x = std::max({ current_velocity.linear.x - control_duration_ * max_x_accel_, -max_x_vel_, -max_stopping_velocity.linear.x });

  const double upper_x = std::min({ current_velocity.linear.x + control_duration_ * max_x_accel_, max_x_vel_, max_stopping_velocity.linear.x });

  const double lower_y = std::max({ current_velocity.linear.y - control_duration_ * max_y_accel_, -max_y_vel_ });

  const double upper_y = std::min({ current_velocity.linear.y + control_duration_ * max_y_accel_, max_y_vel_ });

  // Calculate independent yaw constraints
  const double lower_w = std::max({ current_velocity.angular.z - control_duration_ * max_angular_accel_, -max_angular_vel_, -max_stopping_velocity.angular.z });

  const double upper_w = std::min({ current_velocity.angular.z + control_duration_ * max_angular_accel_, max_angular_vel_, max_stopping_velocity.angular.z });

  // Calculate common scaling factor for x and y
  double scale_xy = 1.0;

  auto updateScale = [&scale_xy](double value, double lower, double upper) {
    if (value > upper && value > 0.0) {
      scale_xy = std::min(scale_xy, upper / value);
    } else if (value < lower && value < 0.0) {
      scale_xy = std::min(scale_xy, lower / value);
    }
  };

  updateScale(raw_velocity.twist.linear.x, lower_x, upper_x);
  updateScale(raw_velocity.twist.linear.y, lower_y, upper_y);

  scale_xy = std::clamp(scale_xy, 0.0, 1.0);

  RCLCPP_INFO(logger_, "scale_xy: %f", scale_xy);

  cmd_vel.twist.linear.x = raw_velocity.twist.linear.x * scale_xy;
  cmd_vel.twist.linear.y = raw_velocity.twist.linear.y * scale_xy;

  // Apply yaw constraints independently
  cmd_vel.twist.angular.z = std::clamp(
    raw_velocity.twist.angular.z,
    lower_w,
    upper_w
  );

  return cmd_vel;
}

geometry_msgs::msg::Twist
MecanumController::computeMaxStoppingVelocity(
  const geometry_msgs::msg::PoseStamped& robot_pose
)
{
  // TODO: L2-Norm
  auto global_goal = path_handler_->getGlobalGoal();
  auto goal_local_frame = transformPoseToRobotFrame(global_goal, robot_pose);

  geometry_msgs::msg::Twist output;

  double distance_x =
    std::abs(robot_pose.pose.position.x - goal_local_frame.pose.position.x);
  output.linear.x = std::sqrt(2.0 * max_x_accel_ * distance_x);

  double distance_y =
    std::abs(robot_pose.pose.position.y - goal_local_frame.pose.position.y);
  output.linear.y = std::sqrt(2.0 * max_y_accel_ * distance_y);

  // Extract yaw angles (in radians)
  double yaw_robot = tf2::getYaw(robot_pose.pose.orientation);
  double yaw_goal = tf2::getYaw(goal_local_frame.pose.orientation);

  // Calculate the shortest angular difference in radians
  double yaw_diff = angles::shortest_angular_distance(yaw_robot, yaw_goal);
  output.angular.z = std::sqrt(2.0 * max_angular_accel_ * yaw_diff);

  return output;
}

geometry_msgs::msg::PoseStamped
MecanumController::transformPoseToRobotFrame(
  const geometry_msgs::msg::PoseStamped& input_pose,
  const geometry_msgs::msg::PoseStamped& robot_pose
)
{
  // Extract robot orientation and position.
  const auto& robot_position = robot_pose.pose.position;
  const auto& robot_orientation = robot_pose.pose.orientation;

  const Eigen::Vector3d robot_translation(
    robot_position.x,
    robot_position.y,
    robot_position.z
  );

  const Eigen::Quaterniond robot_rotation(
    robot_orientation.w,
    robot_orientation.x,
    robot_orientation.y,
    robot_orientation.z
  );

  // Extract input position and orientation.
  const auto& input_position = input_pose.pose.position;
  const auto& input_orientation = input_pose.pose.orientation;

  const Eigen::Vector3d input_translation(
    input_position.x,
    input_position.y,
    input_position.z
  );

  const Eigen::Quaterniond input_rotation(
    input_orientation.w,
    input_orientation.x,
    input_orientation.y,
    input_orientation.z
  );

  // Transform the input pose into the robot coordinate frame.
  const Eigen::Quaterniond inverse_robot_rotation =
    robot_rotation.inverse();

  const Eigen::Vector3d relative_translation =
    inverse_robot_rotation * (input_translation - robot_translation);

  const Eigen::Quaterniond relative_rotation =
    inverse_robot_rotation * input_rotation;

  // Construct the result.
  geometry_msgs::msg::PoseStamped result;
  result.header = input_pose.header;
  result.header.frame_id = robot_pose.header.frame_id;

  result.pose.position.x = relative_translation.x();
  result.pose.position.y = relative_translation.y();
  result.pose.position.z = relative_translation.z();

  result.pose.orientation.x = relative_rotation.x();
  result.pose.orientation.y = relative_rotation.y();
  result.pose.orientation.z = relative_rotation.z();
  result.pose.orientation.w = relative_rotation.w();

  return result;
}

PathHandler::Pose2D MecanumController::calculateError(const geometry_msgs::msg::PoseStamped &robot_pose)
{
  const auto next_target = path_handler_->getLocalGoal(robot_pose);
  const auto local_target = transformPoseToRobotFrame(next_target, robot_pose);

  PathHandler::Pose2D error;

  error.x = local_target.pose.position.x;
  error.y = local_target.pose.position.y;
  error.yaw = tf2::getYaw(local_target.pose.orientation);

  return error;
}

geometry_msgs::msg::TwistStamped MecanumController::calculateRawTwist(const PathHandler::Pose2D &error, const std::string &frame_id)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame_id;
  twist.twist.linear.x = error.x * linear_gain_;
  twist.twist.linear.y = error.y * linear_gain_;
  twist.twist.angular.z = error.yaw * angular_gain_;
  return twist;
}

bool
MecanumController::cancel()
{
  RCLCPP_INFO(logger_, "%s: cancel", __PRETTY_FUNCTION__);
  return true;
}

void
MecanumController::reset()
{
  RCLCPP_INFO(logger_, "%s: reset", __PRETTY_FUNCTION__);
  local_goal_publisher_.reset();
}

void
MecanumController::setSpeedLimit(
  [[maybe_unused]] const double& speed_limit,
  [[maybe_unused]] const bool& percentage
)
{
}

void
MecanumController::loadParameters()
{
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
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".waypoint_tolerance", waypoint_tolerance_);
}

} // namespace edu_fleet::mecanum_path_controller

PLUGINLIB_EXPORT_CLASS(edu_fleet::mecanum_path_controller::MecanumController, nav2_core::Controller);