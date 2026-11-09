#ifndef EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_
#define EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>

namespace edu_fleet::mecanum_path_controller {

/**
 * @brief calculates the euclidean distance (L2-Norm) between 2 geometry_msgs::PoseStamped
 */
double
distanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2);

/**
 * @brief Handles global paths and calculates the next local Goal
 */
class PathHandler
{

public:
  struct GlobalError2D
  {
    Eigen::Vector2d offset;
    Eigen::Vector2d along; // normalized -> only direction relevant
    double yaw; // already in robot frame and does not need to be transformed

    GlobalError2D() = default;
  };

  PathHandler(tf2::Duration transform_tolerance, std::shared_ptr<tf2_ros::Buffer> tf);
  ~PathHandler() = default;

  // TODO: Reject paths that dont contain poses
  void setPlan(const nav_msgs::msg::Path& path)
  {
    global_plan_ = path;
    max_index_ = path.poses.size() - 1;
    nearest_index_initialized_ = false;
  }

  /**
   * @brief calculate the x(along the path)-, y(normal to the path) and yaw error in global map-frame
   */
  GlobalError2D getGlobalError(const geometry_msgs::msg::PoseStamped& robot_pose);
  
  /**
   * @brief calculate the distance along the path from the current robot position to to final goal position
   */
  double getLinearGoalDistance(const geometry_msgs::msg::PoseStamped& robot_pose);

  //  /*  *
  //   * @brief calculate the next local goal position based on the current global position
  //   */
  //   geometry_msgs::msg::PoseStamped getLocalGoal(const geometry_msgs::msg::PoseStamped& global_pose);

  // /**
  // * @brief returns the last position of the path
  // */
  // geometry_msgs::msg::PoseStamped getGlobalGoal();

  // /**
  //  * @brief calculates the max angle between 2 Path segments in the next few segments
  //  * @param lookahead the amount of segments in front of the robot to check
  //  * @param lookback the amount of segments behind the robot to check
  //  */
  // double getMaxAngle(size_t lookahead, size_t lookback);

protected:
  /**
   * @brief find the the index of the pose that is closest to the robot on the whole path. Stores the result in attribute
   * This is only meant to be executed once when a new path is set
   */
  void findNearestIndex(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief increment the nearest robot pose if the robot has passed the pose
   * this is meant to be continuously executed while the robot is driving
   */
  void updateNearestIndex(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief project a given point onto the path segment
   */
  geometry_msgs::msg::Point projectPoint(const geometry_msgs::msg::Point& point, size_t end_index);

  /**
   * @brief compute the factor how far along the segment the projected point is. 0-> at the start; 1 -> at the end; >1 -> outside
   */
  double getProjectionFactor(const geometry_msgs::msg::Point& point, size_t end_index);

  double getAngleError(const geometry_msgs::msg::PoseStamped& robot_pose);

  rclcpp::Logger logger_{ rclcpp::get_logger("edu_PathHandler") };
  tf2::Duration transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav_msgs::msg::Path global_plan_;
  // double waypoint_distance_;
  bool nearest_index_initialized_;
  size_t nearest_index_; // the index of the nearest point on the path that is ahead of the robot; must be >= 1
  size_t max_index_;     // the maximum valid index for the current global plan
};

} // namespace edu_fleet::mecanum_path_controller

#endif // EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_