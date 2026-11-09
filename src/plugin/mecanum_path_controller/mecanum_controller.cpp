#include "edu_fleet/mecanum_path_controller/mecanum_controller.hpp"
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

void
MecanumController::configure(
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
  local_costmap_ = costmap_ros;

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


  // hardcode parameters for testing purposes; TODO: remove
  corner_lookahead_ = 2;
  corner_lookback_ = 3;
  apply_corner_velocity_scaling_ = true;
  min_corner_scaling_factor_ = 0.15;
  start_corner_scaling_angle_ = 0.3;
  end_corner_scaling_angle_ = 1.5;

  offset_gain_ = 2.0;
  angular_gain_ = 2.0;
  along_gain_ = 0.7;
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

geometry_msgs::msg::TwistStamped
MecanumController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& velocity,
  [[maybe_unused]] nav2_core::GoalChecker* /*goal_checker*/
)
{
  auto global_error = path_handler_->getGlobalError(pose);

  // apply gain to global error
  global_error.offset *= offset_gain_; // e_y -> error orthogonal to path 
  global_error.yaw *= angular_gain_; // rotational error

  // experiment of applying gain to error parallel to path instead of the case-based approach below
  global_error.along *= along_gain_;
  if(global_error.along.norm() > max_linear_vel_) // clamp speed to maximum velocity from parameters
  {
    global_error.along.normalize();
    global_error.along *= max_linear_vel_;
  }

  // ensure linear slow-down when approaching the goal
  // double lin_goal_dist = path_handler_->getLinearGoalDistance(pose);
  // if(lin_goal_dist > ((max_linear_vel_ * max_linear_vel_)/(2* max_linear_accel_)))
  // { 
  //   global_error.along *= max_linear_vel_;
  // }
  // else
  // {
  //   global_error.along *= std::sqrt(2*lin_goal_dist*max_linear_accel_);
  // }

  // ensure angular slow-down to avoid overshooting
  // if(global_error.yaw <= ((max_angular_vel_ * max_angular_vel_)/(2 * max_angular_accel_)))
  // {
  //   global_error.yaw = std::sqrt(2*global_error.yaw * max_angular_accel_);
  // }

  auto twist = convertToTwist(global_error, pose);

  auto scaled_twist = applyScalingFactors(twist, velocity, pose);

  geometry_msgs::msg::TwistStamped stamped;

  stamped.header.frame_id = pose.header.frame_id;
  stamped.twist = scaled_twist;

  return stamped;
}

geometry_msgs::msg::Twist
MecanumController::convertToTwist(const PathHandler::GlobalError2D& global_error, const geometry_msgs::msg::PoseStamped& robot_pose)
{
  geometry_msgs::msg::Twist twist;

  // Extract robot yaw
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // Rotation from global frame to robot frame
  const Eigen::Rotation2Dd global_to_robot(-robot_yaw);

  // Transform vectors into robot frame
  const Eigen::Vector2d offset_robot =
    global_to_robot * global_error.offset;

  const Eigen::Vector2d along_robot =
    global_to_robot * global_error.along;

  // Assign translational components
  twist.linear.x = offset_robot.x() + along_robot.x();
  twist.linear.y = offset_robot.y() + along_robot.y();

  // Yaw is already in robot frame
  twist.angular.z = global_error.yaw;

  return twist;
}

geometry_msgs::msg::Twist MecanumController::applyScalingFactors(const geometry_msgs::msg::Twist &cmd_vel_raw, const geometry_msgs::msg::Twist &current_vel, const geometry_msgs::msg::PoseStamped &robot_pose)
{
  geometry_msgs::msg::Twist scaled = cmd_vel_raw;
  double lowest_linear_factor = 1.0;
  double lowest_angular_factor = 1.0;

  // compute lowest(most restriction) factor
  for(auto function : velocity_factor_functions_)
  {
    auto scale = function(cmd_vel_raw, current_vel, robot_pose);
    if(scale.linear  < lowest_linear_factor)
    {    
      lowest_linear_factor = scale.linear;
    }
    if(scale.angular < lowest_angular_factor)
    {
      lowest_angular_factor = scale.angular;
    }
  }

  // apply scale
  scaled.linear.x *= lowest_linear_factor;
  scaled.linear.y *= lowest_linear_factor;
  scaled.angular.z *= lowest_angular_factor;

  return scaled;
}

VelocityScale MecanumController::computeAccelerationLimitFactor(const geometry_msgs::msg::Twist & cmd_vel_raw, const geometry_msgs::msg::Twist & current_vel, const geometry_msgs::msg::PoseStamped &)
{
  /*
  new_max = current + dt * accel
  new_max = cmd_raw * factor <-> factor = new_max/cmd_raw 
  */
  VelocityScale scale;
  
  // angular acceleration
  double max_new_angular_vel = current_vel.angular.z + control_duration_ * max_angular_accel_;
  double inter = std::abs(max_new_angular_vel / cmd_vel_raw.angular.z);
  scale.angular = std::clamp(inter, 0.0, 1.0);
  RCLCPP_INFO(logger_, "max_new_angular_vel: %f; inter: %f, scale_factor: %f", max_new_angular_vel, inter, scale.angular);

  // linear acceleration
  double current_vel_vector_length = std::sqrt(current_vel.linear.x * current_vel.linear.x + current_vel.linear.y * current_vel.linear.y);
  double cmd_vel_vector_length = std::sqrt(cmd_vel_raw.linear.x * cmd_vel_raw.linear.x + cmd_vel_raw.linear.y * cmd_vel_raw.linear.y);

  double max_new_linear_vel = current_vel_vector_length + control_duration_ * max_linear_accel_;
  scale.linear = std::clamp(max_new_linear_vel/cmd_vel_vector_length, 0.0, 1.0);


  // RCLCPP_INFO(logger_, "max_new_linear_vel: %f, current_vel_vector_length: %f, cmd_vel_vector_length: %f", max_new_linear_vel, current_vel_vector_length, cmd_vel_vector_length);
  // RCLCPP_INFO(logger_, "accel-limit: linear: %f, angular: %f", scale.linear, scale.angular);
  // RCLCPP_INFO(logger_, "current_vel: (%f, %f, %f)", current_vel.linear.x,current_vel.linear.y, current_vel.angular.z);
  
  return scale;
}

VelocityScale MecanumController::computeCornerScalingFactor(const geometry_msgs::msg::Twist &cmd_vel_raw, const geometry_msgs::msg::Twist &current_vel, const geometry_msgs::msg::PoseStamped &robot_pose)
{
  /*
  - compute max velocity based on how tight the corner is
  - use deceleration function to ensure low speed is reached in time
  */

  VelocityScale scale;

  RCLCPP_INFO(logger_, "corner scaling");

  return scale;
}

VelocityScale MecanumController::computeCollisionAvoidanceScale(
  const geometry_msgs::msg::Twist &cmd_vel_raw,
  const geometry_msgs::msg::Twist &,
  const geometry_msgs::msg::PoseStamped &robot_pose)
{
  VelocityScale scale;


  return scale;
}


// void
// MecanumController::applyCornerSpeedScaling(geometry_msgs::msg::TwistStamped& raw_velocity)
// {
//   const double max_angle = path_handler_->getMaxAngle(corner_lookahead_, corner_lookback_);
//   double scaling_factor;

//   if (max_angle < start_corner_scaling_angle_) {
//     // skip for straight ish paths
//     RCLCPP_INFO(logger_, "corner angle (%f) blow threshold -> no scaling", max_angle);
//     return;
//   }
//   if (max_angle > end_corner_scaling_angle_) // stop at max value to prevent negative scaling etc.
//     scaling_factor = min_corner_scaling_factor_;
//   else {
//     scaling_factor = 1 + (((min_corner_scaling_factor_ - 1) / (end_corner_scaling_angle_ - start_corner_scaling_angle_)) * (max_angle - start_corner_scaling_angle_));
//   }

//   raw_velocity.twist.linear.x *= scaling_factor;
//   raw_velocity.twist.linear.y *= scaling_factor;
// }

// geometry_msgs::msg::TwistStamped
// MecanumController::applyMotionConstraints(
//   const geometry_msgs::msg::TwistStamped& raw_velocity,
//   const geometry_msgs::msg::Twist& current_velocity,
//   const geometry_msgs::msg::Twist& max_stopping_velocity
// )
// {
//   geometry_msgs::msg::TwistStamped cmd_vel;
//   cmd_vel.header = raw_velocity.header;

//   // Calculate translational constraints
//   const double lower_x = std::max({ current_velocity.linear.x - control_duration_ * max_x_accel_, -max_x_vel_, -max_stopping_velocity.linear.x });

//   const double upper_x = std::min({ current_velocity.linear.x + control_duration_ * max_x_accel_, max_x_vel_, max_stopping_velocity.linear.x });

//   const double lower_y = std::max({ current_velocity.linear.y - control_duration_ * max_y_accel_, -max_y_vel_ });

//   const double upper_y = std::min({ current_velocity.linear.y + control_duration_ * max_y_accel_, max_y_vel_ });

//   // Calculate independent yaw constraints
//   const double lower_w = std::max({ current_velocity.angular.z - control_duration_ * max_angular_accel_, -max_angular_vel_, -max_stopping_velocity.angular.z });

//   const double upper_w = std::min({ current_velocity.angular.z + control_duration_ * max_angular_accel_, max_angular_vel_, max_stopping_velocity.angular.z });

//   // Calculate common scaling factor for x and y
//   double scale_xy = 1.0;

//   auto updateScale = [&scale_xy](double value, double lower, double upper) {
//     if (value > upper && value > 0.0) {
//       scale_xy = std::min(scale_xy, upper / value);
//     } else if (value < lower && value < 0.0) {
//       scale_xy = std::min(scale_xy, lower / value);
//     }
//   };

//   updateScale(raw_velocity.twist.linear.x, lower_x, upper_x);
//   updateScale(raw_velocity.twist.linear.y, lower_y, upper_y);

//   scale_xy = std::clamp(scale_xy, 0.0, 1.0);

//   RCLCPP_INFO(logger_, "scale_xy: %f", scale_xy);

//   cmd_vel.twist.linear.x = raw_velocity.twist.linear.x * scale_xy;
//   cmd_vel.twist.linear.y = raw_velocity.twist.linear.y * scale_xy;

//   // Apply yaw constraints independently
//   cmd_vel.twist.angular.z = std::clamp(
//     raw_velocity.twist.angular.z,
//     lower_w,
//     upper_w
//   );

//   return cmd_vel;
// }

// geometry_msgs::msg::Twist
// MecanumController::computeMaxStoppingVelocity(
//   const geometry_msgs::msg::PoseStamped& robot_pose
// )
// {
//   // TODO: L2-Norm
//   auto global_goal = path_handler_->getGlobalGoal();
//   auto goal_local_frame = transformPoseToRobotFrame(global_goal, robot_pose);

//   geometry_msgs::msg::Twist output;

//   double distance_x =
//     std::abs(robot_pose.pose.position.x - goal_local_frame.pose.position.x);
//   output.linear.x = std::sqrt(2.0 * max_x_accel_ * distance_x);

//   double distance_y =
//     std::abs(robot_pose.pose.position.y - goal_local_frame.pose.position.y);
//   output.linear.y = std::sqrt(2.0 * max_y_accel_ * distance_y);

//   // Extract yaw angles (in radians)
//   double yaw_robot = tf2::getYaw(robot_pose.pose.orientation);
//   double yaw_goal = tf2::getYaw(goal_local_frame.pose.orientation);

//   // Calculate the shortest angular difference in radians
//   double yaw_diff = angles::shortest_angular_distance(yaw_robot, yaw_goal);
//   output.angular.z = std::sqrt(2.0 * max_angular_accel_ * yaw_diff);

//   return output;
// }

// geometry_msgs::msg::PoseStamped
// MecanumController::transformPoseToRobotFrame(
//   const geometry_msgs::msg::PoseStamped& input_pose,
//   const geometry_msgs::msg::PoseStamped& robot_pose
// )
// {
//   // Extract robot orientation and position.
//   const auto& robot_position = robot_pose.pose.position;
//   const auto& robot_orientation = robot_pose.pose.orientation;

//   const Eigen::Vector3d robot_translation(
//     robot_position.x,
//     robot_position.y,
//     robot_position.z
//   );

//   const Eigen::Quaterniond robot_rotation(
//     robot_orientation.w,
//     robot_orientation.x,
//     robot_orientation.y,
//     robot_orientation.z
//   );

//   // Extract input position and orientation.
//   const auto& input_position = input_pose.pose.position;
//   const auto& input_orientation = input_pose.pose.orientation;

//   const Eigen::Vector3d input_translation(
//     input_position.x,
//     input_position.y,
//     input_position.z
//   );

//   const Eigen::Quaterniond input_rotation(
//     input_orientation.w,
//     input_orientation.x,
//     input_orientation.y,
//     input_orientation.z
//   );

//   // Transform the input pose into the robot coordinate frame.
//   const Eigen::Quaterniond inverse_robot_rotation =
//     robot_rotation.inverse();

//   const Eigen::Vector3d relative_translation =
//     inverse_robot_rotation * (input_translation - robot_translation);

//   const Eigen::Quaterniond relative_rotation =
//     inverse_robot_rotation * input_rotation;

//   // Construct the result.
//   geometry_msgs::msg::PoseStamped result;
//   result.header = input_pose.header;
//   result.header.frame_id = robot_pose.header.frame_id;

//   result.pose.position.x = relative_translation.x();
//   result.pose.position.y = relative_translation.y();
//   result.pose.position.z = relative_translation.z();

//   result.pose.orientation.x = relative_rotation.x();
//   result.pose.orientation.y = relative_rotation.y();
//   result.pose.orientation.z = relative_rotation.z();
//   result.pose.orientation.w = relative_rotation.w();

//   return result;
// }

// PathHandler::GlobalError2D MecanumController::calculateError(const geometry_msgs::msg::PoseStamped &robot_pose)
// {
//   const auto next_target = path_handler_->getLocalGoal(robot_pose);
//   const auto local_target = transformPoseToRobotFrame(next_target, robot_pose);

//   PathHandler::GlobalError2D error;

//   error.x = local_target.pose.position.x;
//   error.y = local_target.pose.position.y;
//   error.yaw = tf2::getYaw(local_target.pose.orientation);

//   return error;
// }

// geometry_msgs::msg::TwistStamped MecanumController::calculateRawTwist(const PathHandler::GlobalError2D &error, const std::string &frame_id)
// {
//   geometry_msgs::msg::TwistStamped twist;
//   twist.header.frame_id = frame_id;
//   twist.twist.linear.x = error.x * linear_gain_;
//   twist.twist.linear.y = error.y * linear_gain_;
//   twist.twist.angular.z = error.yaw * angular_gain_;
//   return twist;
// }

// MecanumController::VelocityScale MecanumController::computeDistToGoalScale(const geometry_msgs::msg::PoseStamped &robot_pose, const geometry_msgs::msg::Twist &current_velocity, const geometry_msgs::msg::TwistStamped &cmd_vel_raw)
// {

// }

void
MecanumController::loadParameters()
{
  auto node = node_.lock();

  node->declare_parameter<double>(plugin_name_ + ".max_linear_vel", 0.25);
  node->declare_parameter<double>(plugin_name_ + ".max_angular_vel", 1.0);
  node->declare_parameter<double>(plugin_name_ + ".max_linear_accel", 0.1);
  node->declare_parameter<double>(plugin_name_ + ".max_angular_accel", 0.4);
  node->declare_parameter<double>(plugin_name_ + ".transform_tolerance", 1.0);
  node->declare_parameter<double>(plugin_name_ + ".waypoint_tolerance", 0.25);

  double control_frequency = 20;
  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".waypoint_tolerance", waypoint_tolerance_);


  // decide which velocity scaling functions to use
  bool accel_scaling, corner_scaling;
  node->declare_parameter<bool>(plugin_name_ + ".do_acceleration_scaling", true);
  node->declare_parameter<bool>(plugin_name_ + ".corner_velocity_scaling.apply", true);

  node->get_parameter(plugin_name_ + ".do_acceleration_scaling", accel_scaling);
  node->get_parameter(plugin_name_ + ".corner_velocity_scaling.apply", corner_scaling);
  
  if(accel_scaling == true)
  {
    velocity_factor_functions_.push_back(std::bind(
        &MecanumController::computeAccelerationLimitFactor,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
    ));
    RCLCPP_INFO(logger_, "Activating velocity Scaling based on acceleration!");
  }

  if(corner_scaling == true)
  {
    velocity_factor_functions_.push_back(std::bind(
        &MecanumController::computeCornerScalingFactor,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
    ));
    RCLCPP_INFO(logger_, "Activating velocity Scaling based on Corners!");
  }


}

} // namespace edu_fleet::mecanum_path_controller

PLUGINLIB_EXPORT_CLASS(edu_fleet::mecanum_path_controller::MecanumController, nav2_core::Controller);