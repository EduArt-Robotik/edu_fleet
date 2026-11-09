#ifndef EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_
#define EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_

#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include <memory>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace edu_fleet::mecanum_path_controller {

struct VelocityScale
{
  double linear, angular;
  VelocityScale(): linear(1.0), angular(1.0){};
  VelocityScale(double lin, double angle): linear(lin), angular(angle){};
};

using VelocityFactorFunction = std::function<
  VelocityScale(
    const geometry_msgs::msg::Twist&,
    const geometry_msgs::msg::Twist&,
    const geometry_msgs::msg::PoseStamped&
  )>;

class MecanumController : public nav2_core::Controller
{
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
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  ) override;

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
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path& path) override;

  bool cancel() override;

  void reset() override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

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
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity, nav2_core::GoalChecker* /*goal_checker*/) override;

protected:
  geometry_msgs::msg::Twist convertToTwist(const PathHandler::GlobalError2D& global_error, const geometry_msgs::msg::PoseStamped& robot_pose);

  geometry_msgs::msg::Twist applyScalingFactors(const geometry_msgs::msg::Twist& cmd_vel_raw, const geometry_msgs::msg::Twist& current_vel, const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief compute the velocity scaling factor to ensure acceleration limits
   */
  VelocityScale computeAccelerationLimitFactor(const geometry_msgs::msg::Twist& cmd_vel_raw, const geometry_msgs::msg::Twist& current_vel, const geometry_msgs::msg::PoseStamped& robot_pose);
  
  /**
   * @brief compute the velocity scaling factor to ensure acceleration limits
   */
  VelocityScale computeCornerScalingFactor(const geometry_msgs::msg::Twist& cmd_vel_raw, const geometry_msgs::msg::Twist& current_vel, const geometry_msgs::msg::PoseStamped& robot_pose);
  
  
  // /**
  //  * @brief apply a scaling factor to the raw speed based the on how steep the next corners are going to be
  //  */
  // void applyCornerSpeedScaling(geometry_msgs::msg::TwistStamped& raw_velocity);

  // geometry_msgs::msg::TwistStamped applyMotionConstraints(
  //   const geometry_msgs::msg::TwistStamped& raw_velocity,
  //   const geometry_msgs::msg::Twist& current_velocity,
  //   const geometry_msgs::msg::Twist& max_stopping_velocity
  // );

  // /**
  //  * @brief computes the maximum velocity so that the robot is able to stop at
  //  * the Goal and ont overshoot
  //  */
  // geometry_msgs::msg::Twist
  // computeMaxStoppingVelocity(const geometry_msgs::msg::PoseStamped& robot_pose);

  // geometry_msgs::msg::PoseStamped transformPoseToRobotFrame(
  //   const geometry_msgs::msg::PoseStamped& input_pose,
  //   const geometry_msgs::msg::PoseStamped& robot_pose
  // );

  // /**
  // * @brief calculate the x,y and yaw offset from the robot to the local Goal
  // */
  // PathHandler::GlobalError2D calculateError(const geometry_msgs::msg::PoseStamped & robot_pose);

  // /**
  // * @brief convert the error into twist with P-Controller
  // */
  // geometry_msgs::msg::TwistStamped calculateRawTwist(const PathHandler::GlobalError2D & error, const std::string &frame_id);

  // /**
  // * compute the linear and angular scaling factor based on the distance to the final goal
  // */
  // VelocityScale computeDistToGoalScale(
  //   const geometry_msgs::msg::PoseStamped & robot_pose,
  //   const geometry_msgs::msg::Twist & current_velocity,
  //   const geometry_msgs::msg::TwistStamped & cmd_vel_raw
  // );

  void loadParameters();

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{ rclcpp::get_logger("MecanumController") };
  std::shared_ptr<edu_fleet::mecanum_path_controller::PathHandler>
    path_handler_;
  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_publisher_;

  std::vector<VelocityFactorFunction> velocity_factor_functions_;

  double control_duration_;
  double max_linear_vel_, max_angular_vel_, max_linear_accel_, max_angular_accel_;
  double offset_gain_, angular_gain_;
  double transform_tolerance_, waypoint_tolerance_;
  // Corner Scaling
  bool apply_corner_velocity_scaling_;
  double min_corner_scaling_factor_, start_corner_scaling_angle_, end_corner_scaling_angle_;
  int corner_lookahead_, corner_lookback_;
};

} // namespace edu_fleet::mecanum_path_controller

#endif // EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_