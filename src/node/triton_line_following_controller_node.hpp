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
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <accerion_driver_msgs/srv/set_cluster_mode.hpp>

#include <sick_lidar_localization_msgs/msg/code_measurement_message0304.hpp>

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
    double docking_end_error = 0.07;
    double v_x = 0.1;  // constant forward velocity during line following
    std::chrono::milliseconds stop_time{5000};  // time to stop at the end position, before docking out
    std::string target_frame_id = "eduard/blue/base_link";
    std::string sensor_frame_id = "eduard/blue/triton";
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
  void callbackCode(std::shared_ptr<const sick_lidar_localization_msgs::msg::CodeMeasurementMessage0304> msg);
  void determineDockingState(const double error_x);
  void enableLineFollowingMode(const std::uint8_t cluster_id);
  void disableLineFollowingMode(const std::uint8_t cluster_id);

  const Parameter _parameter;

  std::shared_ptr<controller::Pid> _pid_y;
  std::shared_ptr<controller::Pid> _pid_heading;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> _pub_twist;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseArray>> _sub_line_following;
  std::shared_ptr<rclcpp::Subscription<sick_lidar_localization_msgs::msg::CodeMeasurementMessage0304>> _sub_code;
  std::shared_ptr<rclcpp::Client<accerion_driver_msgs::srv::SetClusterMode>> _client_set_line_following;
  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

  struct {
    rclcpp::Time stamp_last_processing;
    rclcpp::Time stamp_endposition_reached;
    bool docking_in = false;
    bool docking_out = false;
    bool at_endposition = false;
    std::uint8_t active_cluster_id = 8; //> 0 means no cluster selected
  } _data;
};

} // end namespace fleet
} // end namespace eduart
