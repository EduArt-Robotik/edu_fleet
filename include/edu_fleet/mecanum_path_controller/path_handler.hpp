#ifndef EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_
#define EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_

#include "edu_fleet/mecanum_path_controller/costmap_handler.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <memory>

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
    Eigen::Vector2d projection; // error orthogonal to the path
    Eigen::Vector2d along; // normalized -> only direction relevant
    double yaw; // already in robot frame and does not need to be transformed

    GlobalError2D() = default;
  };

  PathHandler(tf2::Duration transform_tolerance, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<CostmapHandler> costmap_handler);
  ~PathHandler() = default;

  // TODO: Reject paths that dont contain poses
  void setPlan(const nav_msgs::msg::Path& path);

  /**
   * @brief calculate the x(along the path)-, y(normal to the path) and yaw error in global map-frame
   */
  GlobalError2D getGlobalError(const geometry_msgs::msg::PoseStamped& robot_pose);
  
  /**
   * @brief calculate the distance along the path from the current robot position to to final goal position
   */
  double getLinearGoalDistance(const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
  * @brief lookahead on the path and return a reduced speed if there are close objects or collisions 
  */
  double collisionLookahead(const geometry_msgs::msg::PoseStamped & robot_pose, const double lookahead_distance, const unsigned char cost_max);

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
   * @param end_index the index to the point at the end of the path segment
   */
  geometry_msgs::msg::Point projectPoint(const geometry_msgs::msg::Point& point, size_t end_index);

  /**
   * @brief compute the factor how far along the segment the projected point is. 0-> at the start; 1 -> at the end; >1 -> outside
   */
  double getProjectionFactor(const geometry_msgs::msg::Point& point, size_t end_index);

  double getAngleError(const geometry_msgs::msg::PoseStamped& robot_pose);

  tf2::Duration transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav_msgs::msg::Path global_plan_;

  std::shared_ptr<CostmapHandler> costmap_handler_;
  rclcpp::Logger logger_{ rclcpp::get_logger("edu_PathHandler") };

  bool nearest_index_initialized_;
  size_t nearest_index_; // the index of the nearest point on the path that is ahead of the robot; must be >= 1
  size_t max_index_;     // the maximum valid index for the current global plan
};

} // namespace edu_fleet::mecanum_path_controller

#endif // EDU_FLEET__MECANUM_PATH_CONTROLLER__PATH_HANDLER_HPP_