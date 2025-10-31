 /**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/controller/pid.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/pose_array.hpp>

namespace eduart {
namespace fleet {

class TritonLineFollowingController : public rclcpp_lifecycle::LifecycleNode
{
public:
  static inline constexpr std::size_t NUM_SENSORS = 3;

  struct Parameter {
    
    struct {
      controller::Pid::Parameter y = {
         1.0, 0.0, 0.0, 0.5, 1.0, true};
      controller::Pid::Parameter heading = {
         1.0, 0.0, 0.0, M_PI_2, 1.0, true};
    } pid;
  };

  TritonLineFollowingController();
  ~TritonLineFollowingController() override = default;

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp_lifecycle::LifecycleNode& ros_node);

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State& previous_state) override;

private:
  void callbackLineFollowingPoses(std::shared_ptr<const geometry_msgs::msg::PoseArray> msg);

  const Parameter _parameter;

  std::shared_ptr<controller::Pid> _pid_y;
  std::shared_ptr<controller::Pid> _pid_heading;
};

} // end namespace fleet
} // end namespace eduart
