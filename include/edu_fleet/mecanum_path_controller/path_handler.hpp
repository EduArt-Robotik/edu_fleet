#ifndef EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_
#define EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_

#include <cstddef>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace edu_fleet::mecanum_path_controller {

/**
 * @brief Handles global paths and calculates the next local Goal
 */
class PathHandler {

public:

  struct Pose2D
  {
    double x, y, yaw;

    Pose2D(double x, double y, double yaw): x(x), y(y), yaw(yaw){};
    Pose2D() = default;
  };


  /**
   * @param waypoint_distance: the maximal distance to a waypoint on the path to
   * move on to the next waypoint
   */
  PathHandler(tf2::Duration transform_tolerance,
              std::shared_ptr<tf2_ros::Buffer> tf, double waypoint_distance);
  ~PathHandler() = default;

  // TODO: Reject paths that dont contain poses
  void setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; current_index_ = 0;}

  /**
  * @brief calculate the next local goal position based on the current global position
  */
  geometry_msgs::msg::PoseStamped getLocalGoal(const geometry_msgs::msg::PoseStamped& global_pose);

  /**
  * @brief returns the last position of the path
  */
  geometry_msgs::msg::PoseStamped getGlobalGoal();

  /**
  * @brief calculates the max angle between 2 Path segments in the next few segments
  * @param lookahead the amount of segments in front of the robot to check
  * @param lookback the amount of segments behind the robot to check
  */
  double getMaxAngle(size_t lookahead, size_t lookback);

protected:
  rclcpp::Logger logger_{rclcpp::get_logger("edu_PathHandler")};
  tf2::Duration transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav_msgs::msg::Path global_plan_;
  double waypoint_distance_;
  size_t current_index_;
};

} // namespace edu_fleet::mecanum_path_controller

#endif // EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_