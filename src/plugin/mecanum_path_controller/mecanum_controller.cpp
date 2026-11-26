#include "edu_fleet/mecanum_path_controller/mecanum_controller.hpp"
#include "edu_fleet/mecanum_path_controller/costmap_handler.hpp"
#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <Eigen/src/Core/Matrix.h>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

namespace edu_fleet::mecanum_path_controller {

MecanumController::Parameter MecanumController::loadParameters(const std::string prefix, const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  Parameter param;
  // Declare parameters with default values first
  node->declare_parameter(prefix + "transform_tolerance", param.transform_tolerance);
  node->declare_parameter(prefix + "max_linear_vel", param.max_linear_vel);
  node->declare_parameter(prefix + "max_angular_vel", param.max_angular_vel);
  node->declare_parameter(prefix + "max_linear_accel", param.max_linear_accel);
  node->declare_parameter(prefix + "max_angular_accel", param.max_angular_accel);
  node->declare_parameter(prefix + "projection_gain", param.projection_gain);
  node->declare_parameter(prefix + "angular_gain", param.angular_gain);
  node->declare_parameter(prefix + "along_gain", param.along_gain);
  node->declare_parameter(prefix + "controller_frequency", 1/param.control_duration);

  // Then get the parameters from the node
  node->get_parameter(prefix + "transform_tolerance", param.transform_tolerance);
  node->get_parameter(prefix + "max_linear_vel", param.max_linear_vel);
  node->get_parameter(prefix + "max_angular_vel", param.max_angular_vel);
  node->get_parameter(prefix + "max_linear_accel", param.max_linear_accel);
  node->get_parameter(prefix + "max_angular_accel", param.max_angular_accel);
  node->get_parameter(prefix + "projection_gain", param.projection_gain);
  node->get_parameter(prefix + "angular_gain", param.angular_gain);
  node->get_parameter(prefix + "along_gain", param.along_gain);
  param.control_duration = 1.0/node->get_parameter("controller_frequency").as_double();

  return param;
}

void MecanumController::loadScalingFunctions(
  const std::string prefix,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  velocity_factor_functions_.clear();

  node->declare_parameter<bool>(prefix + "acceleration_velocity_scaling.enable", true);
  node->declare_parameter<bool>(prefix + "collision_velocity_scaling.enable", true);

  if(node->get_parameter(prefix + "acceleration_velocity_scaling.enable").as_bool()) {
    velocity_factor_functions_.push_back(std::bind(
        &MecanumController::computeAccelerationLimitFactor,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
    ));
    RCLCPP_INFO(logger_, ANSI_COLOR_GREEN "Activating velocity Scaling based on acceleration!" ANSI_COLOR_RESET);
  }

  if(node->get_parameter(prefix + "collision_velocity_scaling.enable").as_bool()) {
    // load params for collision scaling
    node->declare_parameter(prefix + "collision_velocity_scaling.lookahead", collision_param_.lookahead);
    node->declare_parameter(prefix + "collision_velocity_scaling.stop_cost", collision_param_.cost_max);
    node->get_parameter(prefix + "collision_velocity_scaling.lookahead", collision_param_.lookahead);
    node->get_parameter(prefix + "collision_velocity_scaling.stop_cost", collision_param_.cost_max);  

    velocity_factor_functions_.push_back(std::bind(
        &MecanumController::computePathCollisionFactor,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
    ));
    RCLCPP_INFO(logger_, ANSI_COLOR_GREEN "Activating velocity Scaling based on collision!" ANSI_COLOR_RESET);
  }
}


void MecanumController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
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

  param_ = loadParameters(plugin_name_ + ".", node);
  loadScalingFunctions(plugin_name_ + ".", node);

  costmap_handler_ = std::make_shared<CostmapHandler>(costmap_ros);
  path_handler_ = std::make_shared<edu_fleet::mecanum_path_controller::PathHandler>(
      tf2::durationFromSec(param_.transform_tolerance), tf_, costmap_handler_
  );
}

void MecanumController::cleanup()
{
  RCLCPP_INFO(logger_, 
    "Cleaning up controller: %s of type edu_fleet::mecanum_path_controller::MecanumController",
    plugin_name_.c_str());
}

void MecanumController::activate()
{
  RCLCPP_INFO(logger_, 
    "Activating controller: %s of type edu_fleet::mecanum_path_controller::MecanumController",
    plugin_name_.c_str());
}

void MecanumController::deactivate()
{
  RCLCPP_INFO(logger_, 
  "Deactivating controller: %s of type edu_fleet::mecanum_path_controller::MecanumController",
  plugin_name_.c_str());
}

void MecanumController::setPlan(const nav_msgs::msg::Path& path)
{
  path_handler_->setPlan(path);
}

bool MecanumController::cancel()
{
  RCLCPP_INFO(logger_, "%s: cancel", __PRETTY_FUNCTION__);
  return true;
}

void MecanumController::reset()
{
  RCLCPP_INFO(logger_, "%s: reset", __PRETTY_FUNCTION__);
}

void MecanumController::setSpeedLimit(
  [[maybe_unused]] const double& speed_limit,
  [[maybe_unused]] const bool& percentage
)
{
}

geometry_msgs::msg::TwistStamped MecanumController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& velocity,
  nav2_core::GoalChecker* )
{
  // calculate error in global map frame
  const auto global_error = path_handler_->getGlobalError(pose);

  // apply gain to global error
  PathHandler::GlobalError2D control_command;
  control_command.projection = global_error.projection * param_.projection_gain; // e_y -> error orthogonal to path 
  control_command.yaw = global_error.yaw * param_.angular_gain; // rotational error
  control_command.along = global_error.along * param_.along_gain; // e_x -> distance to goal along the path
  if(control_command.along.norm() > param_.max_linear_vel) 
  { // clamp speed to maximum velocity from parameters
    control_command.along.normalize();
    control_command.along *= param_.max_linear_vel;
  }

  const auto twist = convertToTwist(control_command, pose);

  const auto scaled_twist = applyScalingFactors(twist, velocity, pose);

  geometry_msgs::msg::TwistStamped stamped;

  stamped.header.frame_id = pose.header.frame_id;
  stamped.twist = scaled_twist;

  return stamped;
}

geometry_msgs::msg::Twist MecanumController::convertToTwist(
  const PathHandler::GlobalError2D& global_error,
  const geometry_msgs::msg::PoseStamped& robot_pose)
{
  geometry_msgs::msg::Twist twist;

  // Extract robot yaw
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // Rotation Matrix from global frame to robot frame
  const Eigen::Rotation2Dd global_to_robot(-robot_yaw);

  // Transform vectors into robot frame
  const Eigen::Vector2d projection_robot = global_to_robot * global_error.projection;
  const Eigen::Vector2d along_robot = global_to_robot * global_error.along;

  // Assign translational components
  twist.linear.x = projection_robot.x() + along_robot.x();
  twist.linear.y = projection_robot.y() + along_robot.y();

  // Yaw is already in robot frame
  twist.angular.z = global_error.yaw;

  return twist;
}

geometry_msgs::msg::Twist MecanumController::applyScalingFactors(
  const geometry_msgs::msg::Twist &cmd_vel_raw,
  const geometry_msgs::msg::Twist &current_vel,
  const geometry_msgs::msg::PoseStamped &robot_pose)
{
  geometry_msgs::msg::Twist scaled = cmd_vel_raw;
  double lowest_linear_factor = 1.0;
  double lowest_angular_factor = 1.0;

  // compute lowest(most restrictive) factor
  for(auto function : velocity_factor_functions_)
  {
    auto scale = function(cmd_vel_raw, current_vel, robot_pose);
    lowest_linear_factor = std::min(lowest_linear_factor, scale.linear);
    lowest_angular_factor = std::min(lowest_angular_factor, scale.angular);
  }

  // apply scale
  scaled.linear.x *= lowest_linear_factor;
  scaled.linear.y *= lowest_linear_factor;
  scaled.angular.z *= lowest_angular_factor;

  return scaled;
}

VelocityScale MecanumController::computeAccelerationLimitFactor(
  const geometry_msgs::msg::Twist & cmd_vel_raw,
  const geometry_msgs::msg::Twist & current_vel,
  const geometry_msgs::msg::PoseStamped &)
{
  /*
  new_max = current + dt * accel
  new_max = cmd_raw * factor <-> factor = new_max/cmd_raw 
  */
  VelocityScale scale;
  
  // angular acceleration
  int sign = current_vel.angular.z > 0 ? 1 : -1;
  double max_new_angular_vel = current_vel.angular.z + sign * param_.control_duration * param_.max_angular_accel;
  scale.angular = std::clamp(std::abs(max_new_angular_vel / cmd_vel_raw.angular.z), 0.0, 1.0);

  // linear acceleration
  double current_vel_vector_length = std::sqrt(current_vel.linear.x * current_vel.linear.x + current_vel.linear.y * current_vel.linear.y);
  double cmd_vel_vector_length = std::sqrt(cmd_vel_raw.linear.x * cmd_vel_raw.linear.x + cmd_vel_raw.linear.y * cmd_vel_raw.linear.y);

  double max_new_linear_vel = current_vel_vector_length + param_.control_duration * param_.max_linear_accel;
  scale.linear = std::clamp(max_new_linear_vel/cmd_vel_vector_length, 0.0, 1.0);

  return scale;
}

VelocityScale MecanumController::computePathCollisionFactor(
  const geometry_msgs::msg::Twist &cmd_vel_raw,
  const geometry_msgs::msg::Twist &current_vel,
  const geometry_msgs::msg::PoseStamped &robot_pose)
{
  // desired factor
  const double factor = path_handler_->collisionLookahead(robot_pose, collision_param_.lookahead, collision_param_.cost_max);
  // minimal factor that does not violate deceleration limits
  const auto min_scaling_factor = computeMinDecelerationScalingFactor(cmd_vel_raw, current_vel);
  
  // use the larger of the two factors to respect deceleration limits
  VelocityScale scale;
  scale.angular = std::max(min_scaling_factor.angular, factor);
  scale.linear = std::max(min_scaling_factor.linear, factor);

  return scale;
}

VelocityScale MecanumController::computeMinDecelerationScalingFactor(
  const geometry_msgs::msg::Twist &cmd_vel_raw,
  const geometry_msgs::msg::Twist &current_vel)
{
  /*
  new_min = current - dt * decel
  new_min = cmd_raw * factor <-> factor = new_min/cmd_raw 
  */
  const double raw_linear = std::hypot(cmd_vel_raw.linear.x, cmd_vel_raw.linear.y);
  const double current_linear = std::hypot(current_vel.linear.x, current_vel.linear.y);
  const double raw_angular = std::abs(cmd_vel_raw.angular.z);
  const double current_angular = std::abs(current_vel.angular.z);

  const double min_linear = std::max(0.0, current_linear - param_.control_duration * param_.max_linear_accel);
  const double min_angular = std::max(0.0, current_angular - param_.control_duration * param_.max_angular_accel);
  // clamp to range [0, 1] as scaling factors should not exceed 1 or be negative
  return VelocityScale{
    raw_linear > 0.0 ? std::min(1.0, min_linear / raw_linear) : 0.0,
    raw_angular > 0.0 ? std::min(1.0, min_angular / raw_angular) : 0.0
  };
}


} // namespace edu_fleet::mecanum_path_controller

PLUGINLIB_EXPORT_CLASS(edu_fleet::mecanum_path_controller::MecanumController, nav2_core::Controller);