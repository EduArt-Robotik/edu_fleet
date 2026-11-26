#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "edu_fleet/mecanum_path_controller/costmap_handler.hpp"
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

PathHandler::PathHandler(tf2::Duration transform_tolerance, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<CostmapHandler> costmap_handler)
  : transform_tolerance_(transform_tolerance)
  , tf_(tf)
  , costmap_handler_(costmap_handler)
  {};

void PathHandler::setPlan(const nav_msgs::msg::Path& path)
{
  if(path.poses.empty())
  {
    throw std::invalid_argument("PathHandler::setPlan: path contains no poses");
  }
  global_plan_ = path;
  max_index_ = path.poses.size() - 1;
  nearest_index_initialized_ = false;
}

PathHandler::GlobalError2D PathHandler::getGlobalError(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  // initialize and update nearest index if necessary
  if(nearest_index_initialized_ == false)
  {
    findNearestIndex(robot_pose);
  }
  updateNearestIndex(robot_pose);

  GlobalError2D global_error;

  // project robot position onto the nearest path segment
  const auto projected_point = projectPoint(robot_pose.pose.position, nearest_index_);

  // projection error is the vector from the robot position to the projected point on the path segment
  global_error.projection = Eigen::Vector2d(
    projected_point.x - robot_pose.pose.position.x,
    projected_point.y - robot_pose.pose.position.y
  );

  // along error is the vector along the path segment from the projected point
  global_error.along = Eigen::Vector2d(
    global_plan_.poses[nearest_index_].pose.position.x - global_plan_.poses[nearest_index_-1].pose.position.x,
    global_plan_.poses[nearest_index_].pose.position.y - global_plan_.poses[nearest_index_-1].pose.position.y
  );
  global_error.along.normalize();
  global_error.along *= getLinearGoalDistance(robot_pose); // scale the along error by the remaining distance to the goal

  // handle angle separately
  double angle_error = getAngleError(robot_pose); 
  global_error.yaw = angle_error;

  return global_error;
}

double PathHandler::getLinearGoalDistance(const geometry_msgs::msg::PoseStamped& robot_pose) 
{
  const auto& poses = global_plan_.poses;

  double distance = distanceBetweenPoses(robot_pose, poses[nearest_index_]);

  for (uint i = nearest_index_; i <= max_index_ - 1; ++i) {
    distance += distanceBetweenPoses(poses[i], poses[i + 1]);
  }

  return distance;
}

double PathHandler::collisionLookahead(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double lookahead_distance,
  const unsigned char cost_max)
{
  // start- and endpoints of the path segments; change them while walking through the path
  geometry_msgs::msg::Point start = robot_pose.pose.position;
  geometry_msgs::msg::Point end = projectPoint(robot_pose.pose.position, nearest_index_);
  size_t current_end_index = nearest_index_; // the index to the point that is the end to the current segment
  double distance_remaining = lookahead_distance; // the remaining distance to look ahead; get decremented while traversing the path
  double current_distance = std::hypot(start.x-end.x, start.y-end.y);

  double max_speed_factor = 1.0; // the max allowed speed based on the obstacles ahead

  // step through path segments until the length of the segment is smaller then the remaining lookahead distance
  while(current_distance < distance_remaining && current_end_index <= max_index_)
  {
    distance_remaining -= current_distance;
    max_speed_factor = std::min(max_speed_factor, costmap_handler_->checkSegment(start, end, cost_max));

    // switch the next path segment
    start = end;
    end = global_plan_.poses[current_end_index].pose.position;
    ++current_end_index;

    current_distance = std::hypot(start.x-end.x, start.y-end.y);
  }

  // handle rest of the distance that was not enough for the full segment
  Eigen::Vector2d direction (end.x - start.x, end.y - start.y);
  double last_segment_length = direction.norm();
  direction.normalize();
  direction *= std::min(distance_remaining, last_segment_length); // scale direction vector to correct distance; cap at actual length for the case that the robot approaches the end of the path
  end.x = start.x + direction.x();
  end.y = start.y + direction.y();

  max_speed_factor = std::min(max_speed_factor, costmap_handler_->checkSegment(start, end, cost_max));

  return max_speed_factor;
}

void PathHandler::findNearestIndex(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  // loop through all points to find the one closest to the robot
  nearest_index_ = 0;
  double nearest_distance = distanceBetweenPoses(robot_pose, global_plan_.poses[nearest_index_]);
  for (uint i = 1; i < max_index_; ++i) {
    double distance = distanceBetweenPoses(robot_pose, global_plan_.poses[i]);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index_ = i;
    }
  }
  if(nearest_index_ == 0) // catch case where nearest index is at the start of the path
    nearest_index_ = 1;

  // closes point is either start or end of closest path segment
  // updateNearestIndex will adjust nearest_index_ if closest point is the start of the segment
  updateNearestIndex(robot_pose);
  nearest_index_initialized_ = true;
}

void PathHandler::updateNearestIndex(const geometry_msgs::msg::PoseStamped& robot_pose)
{
  double factor = getProjectionFactor(robot_pose.pose.position, nearest_index_);
  if (factor > 1.0 && nearest_index_ < max_index_) {
    ++nearest_index_;
  }
}

geometry_msgs::msg::Point PathHandler::projectPoint(
  const geometry_msgs::msg::Point& point,
  size_t end_index)
{
  if (end_index < 1 || end_index > max_index_) {
    throw std::runtime_error("end_index must be within [1, poses.size()).");
  }

  // grab the 2 endpoints of the path segment
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

double PathHandler::getProjectionFactor(
  const geometry_msgs::msg::Point& point,
  size_t end_index)
{
  if (end_index < 1 || end_index > max_index_) {
    throw std::runtime_error("end_index must be within [1, poses.size()).");
  }

  // grab the 2 endpoints of the path segment
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


double PathHandler::getAngleError(const geometry_msgs::msg::PoseStamped &robot_pose)
{
  const auto &target = global_plan_.poses[nearest_index_];

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double target_yaw = tf2::getYaw(target.pose.orientation);

  return angles::shortest_angular_distance(robot_yaw, target_yaw);
}

} // namespace edu_fleet::mecanum_path_controller