#ifndef EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_
#define EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_

#include "edu_fleet/mecanum_path_controller/costmap_handler.hpp"
#include "edu_fleet/mecanum_path_controller/path_handler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sys/types.h>

#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

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
  struct Parameter {
    double control_duration = 0.05; // control loop duration in seconds
    double transform_tolerance = 1.0;
    double max_linear_vel = 0.25;
    double max_angular_vel = 1.0;
    double max_linear_accel = 0.5;
    double max_angular_accel = 1.0;

    double along_gain = 0.7; 
    double projection_gain = 2.0;
    double angular_gain = 2.0;
  };
  struct CollisionScalingParameter {
    double lookahead = 0.5;
    unsigned char cost_max = 230; // Stop the robot if the path contains a cost > this value
  };

  static Parameter loadParameters(const std::string prefix, const rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  void loadScalingFunctions(const std::string prefix, const rclcpp_lifecycle::LifecycleNode::SharedPtr node);

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
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker
  ) override;

protected:
  /**
  * @brief Convert the control values in the global path frame to a Twist message in the local robot frame
   */
  geometry_msgs::msg::Twist convertToTwist(const PathHandler::GlobalError2D& control_command, const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
  * @brief Apply scaling factors to raw velocity commands, such as acceleration limits or collision detection
   */
  geometry_msgs::msg::Twist applyScalingFactors(const geometry_msgs::msg::Twist& cmd_vel_raw, const geometry_msgs::msg::Twist& current_vel, const geometry_msgs::msg::PoseStamped& robot_pose);

  /**
   * @brief compute the velocity scaling factor to ensure acceleration limits
   */
  VelocityScale computeAccelerationLimitFactor(
    const geometry_msgs::msg::Twist& cmd_vel_raw,
    const geometry_msgs::msg::Twist& current_vel,
    const geometry_msgs::msg::PoseStamped& robot_pose);
  
  /**
   * @brief compute the velocity scaling factor to stop the robot before hitting an obstacle on the path
   */
  VelocityScale computePathCollisionFactor(
    const geometry_msgs::msg::Twist& cmd_vel_raw,
    const geometry_msgs::msg::Twist& current_vel,
    const geometry_msgs::msg::PoseStamped& robot_pose);

  
  /**
  * @brief compute the minimal scaling factor that can be applied to the raw cmd_vel without violating deceleration limits;
  * assume deceleration is the same as acceleration 
  */
  VelocityScale computeMinDecelerationScalingFactor(
    const geometry_msgs::msg::Twist& cmd_vel_raw,
    const geometry_msgs::msg::Twist& current_vel
  );

  rclcpp::Logger logger_{ rclcpp::get_logger("MecanumController") };

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<PathHandler> path_handler_;
  std::shared_ptr<CostmapHandler> costmap_handler_;

  std::string plugin_name_;
  std::vector<VelocityFactorFunction> velocity_factor_functions_;
  Parameter param_;
  CollisionScalingParameter collision_param_;
};

} // namespace edu_fleet::mecanum_path_controller

#endif // EDU_FLEET__MECANUM_PATH_CONTROLLER__MECANUM_CONTROLLER_HPP_