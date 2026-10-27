#ifndef EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_
#define EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_

#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include <memory>

namespace edu_fleet::mecanum_path_controller {

class MecanumController : public nav2_core::Controller {
public:
  MecanumController() = default;
  ~MecanumController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with
   * possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful
   * in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker * /*goal_checker*/) override;

  bool cancel() override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path &path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

  void reset() override;

protected:
  /**
   * @brief calculates the twist command to the provided goal Position without
   * applying any motion constraints
   */
  geometry_msgs::msg::TwistStamped
  computeRawVelocity(const geometry_msgs::msg::PoseStamped &robot_pose,
                     const geometry_msgs::msg::PoseStamped &goal_pose);

  geometry_msgs::msg::TwistStamped applyMotionConstraints(
      const geometry_msgs::msg::TwistStamped &raw_velocity,
      const geometry_msgs::msg::Twist &current_velocity);

  void loadParameters();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("MecanumController")};
  std::shared_ptr<edu_fleet::mecanum_path_controller::PathHandler> path_handler_;

  double control_duration_;
  double max_x_vel_, max_y_vel_, max_angular_vel_, max_x_accel_, max_y_accel_,
      max_angular_accel_;
  double transform_tolerance_, waypoint_tolerance_;
};

} // namespace edu_fleet::mecanum_path_controller

#endif // EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_