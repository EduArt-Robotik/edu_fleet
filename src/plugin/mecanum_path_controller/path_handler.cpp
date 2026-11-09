#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace edu_fleet::mecanum_path_controller {
double
distanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2)
{
  return std::hypot(
    (pose1.pose.position.x - pose2.pose.position.x),
    (pose1.pose.position.y - pose2.pose.position.y),
    (pose1.pose.position.z - pose2.pose.position.z)
  );
}

PathHandler::PathHandler(tf2::Duration transform_tolerance, std::shared_ptr<tf2_ros::Buffer> tf, double waypoint_distance)
  : transform_tolerance_(transform_tolerance)
  , tf_(tf)
// , waypoint_distance_(waypoint_distance)
{};

PathHandler::GlobalError2D
PathHandler::getGlobalError(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  if(nearest_index_initialized_ == false)
  {
    findNearestIndex(robot_pose);
  }
  updateNearestIndex(robot_pose);

  double angle_error = getAngleError(robot_pose); 

  auto projected_point = projectPoint(robot_pose.pose.position, nearest_index_);

  GlobalError2D global_error;

  global_error.offset = Eigen::Vector2d(
    projected_point.x - robot_pose.pose.position.x,
    projected_point.y - robot_pose.pose.position.y
  );

  global_error.along = Eigen::Vector2d(
    global_plan_.poses[nearest_index_].pose.position.x - global_plan_.poses[nearest_index_-1].pose.position.x,
    global_plan_.poses[nearest_index_].pose.position.y - global_plan_.poses[nearest_index_-1].pose.position.y
  );
  global_error.along.normalize();
  global_error.along *= getLinearGoalDistance(robot_pose);

  global_error.yaw = angle_error;

  // RCLCPP_INFO(logger_, "global error: along: (%f, %f), offset: (%f, %f)", global_error.along.x(), global_error.along.y(), global_error.offset.x(), global_error.offset.y());

  return global_error;
}

// geometry_msgs::msg::PoseStamped
// PathHandler::getLocalGoal(const geometry_msgs::msg::PoseStamped& global_pose)
// {
//   if (current_index_ == global_plan_.poses.size() - 1) {
//     return global_plan_.poses[current_index_];
//   }

//   // due to the check above, ther is always one more pose on the path ahead
//   auto distance = std::hypot(global_pose.pose.position.x - global_plan_.poses[current_index_].pose.position.x, global_pose.pose.position.y - global_plan_.poses[current_index_].pose.position.y);

//   if (distance < waypoint_distance_) {
//     ++current_index_;
//   }

//   return global_plan_.poses[current_index_];
// }

// geometry_msgs::msg::PoseStamped
// PathHandler::getGlobalGoal()
// {
//   if (global_plan_.poses.size() > 0) {
//     return global_plan_.poses[global_plan_.poses.size() - 1];
//   }
//   return geometry_msgs::msg::PoseStamped();
// }

// double
// PathHandler::getMaxAngle(size_t lookahead, size_t lookback)
// {
//   const auto& poses = global_plan_.poses;
//   const size_t path_size = static_cast<size_t>(poses.size());

//   // Include up to two points before the current index
//   const int start_index = current_index_ - std::min(current_index_, lookback);

//   // Do not access points outside the path
//   const int end_index = std::min(path_size - 2, current_index_ + lookahead);

//   double max_angle = 0.0;

//   for (int i = start_index; i < end_index; ++i) {
//     // Calculate the direction vectors of the two consecutive segments
//     const double dx1 = poses[i + 1].pose.position.x - poses[i].pose.position.x;
//     const double dy1 = poses[i + 1].pose.position.y - poses[i].pose.position.y;

//     const double dx2 = poses[i + 2].pose.position.x - poses[i + 1].pose.position.x;
//     const double dy2 = poses[i + 2].pose.position.y - poses[i + 1].pose.position.y;

//     // Calculate the angle between the segments using the dot product
//     const double dot_product = dx1 * dx2 + dy1 * dy2;
//     const double cross_product = dx1 * dy2 - dy1 * dx2;

//     const double angle = std::abs(std::atan2(cross_product, dot_product));

//     // Store the maximum angle
//     max_angle = std::max(max_angle, angle);
//   }

//   return max_angle;
// }

void
PathHandler::findNearestIndex(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  nearest_index_ = 1;
  double nearest_distance = distanceBetweenPoses(robot_pose, global_plan_.poses[nearest_index_]);
  for (uint i = 1; i < max_index_; ++i) {
    double distance = distanceBetweenPoses(robot_pose, global_plan_.poses[i]);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index_ = i;
    }
  }
  updateNearestIndex(robot_pose);
  nearest_index_initialized_ = true;
}

void
PathHandler::updateNearestIndex(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  double factor = getProjectionFactor(robot_pose.pose.position, nearest_index_);
  if (factor > 1.0 && nearest_index_ < max_index_) {
    ++nearest_index_;
  }
}

geometry_msgs::msg::Point
PathHandler::projectPoint(const geometry_msgs::msg::Point& point, size_t end_index)
{
  if (end_index < 1 || end_index > max_index_) {
    throw std::runtime_error("end_index must be within [1, poses.size()).");
  }

  const auto& point1 = global_plan_.poses[end_index - 1].pose.position;
  const auto& point2 = global_plan_.poses[end_index].pose.position;

  // Segment direction vector
  const double dx = point2.x - point1.x;
  const double dy = point2.y - point1.y;

  double t = getProjectionFactor(point, end_index);

  // Clamp to segment
  t = std::clamp(t, 0.0, 1.0);

  // Calculate projected point
  geometry_msgs::msg::Point projected_point;
  projected_point.x = point1.x + t * dx;
  projected_point.y = point1.y + t * dy;

  return projected_point;
}

double
PathHandler::getProjectionFactor(const geometry_msgs::msg::Point& point, size_t end_index)
{
  if (end_index < 1 || end_index > max_index_) {
    throw std::runtime_error("end_index must be within [1, poses.size()).");
  }

  const auto& point1 = global_plan_.poses[end_index - 1].pose.position;
  const auto& point2 = global_plan_.poses[end_index].pose.position;

  // Segment direction vector
  const double dx = point2.x - point1.x;
  const double dy = point2.y - point1.y;

  // Vector from point1 to the input point
  const double px = point.x - point1.x;
  const double py = point.y - point1.y;

  // Squared segment length
  const double length_squared = dx * dx + dy * dy;

  // Handle degenerate segments
  if (length_squared < 1e-12) {
    return 1.0;
  }

  // Projection factor
  return (px * dx + py * dy) / length_squared;
}

double
PathHandler::getLinearGoalDistance(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  const auto& poses = global_plan_.poses;

  double distance = distanceBetweenPoses(robot_pose, poses[nearest_index_]);

  for (uint i = nearest_index_; i < max_index_ - 1; ++i) {
    distance += distanceBetweenPoses(poses[i], poses[i + 1]);
  }

  return distance;
}

double PathHandler::getAngleError(const geometry_msgs::msg::PoseStamped &robot_pose)
{
  const auto &target = global_plan_.poses[nearest_index_];

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double target_yaw = tf2::getYaw(target.pose.orientation);

  return angles::shortest_angular_distance(robot_yaw, target_yaw);
}

} // namespace edu_fleet::mecanum_path_controller