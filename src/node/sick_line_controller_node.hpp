 /**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/controller/pid.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sick_lidar_localization/msg/line_measurement_message0404.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace eduart {
namespace fleet {

class SickLineController : public rclcpp_lifecycle::LifecycleNode
{
public:
  static inline constexpr std::size_t NUM_SENSORS = 3;

  struct Parameter {
    double d_x = 0.25f; // distance between line sensors in meter
    // double gain_yaw = 1.0f; // used to compensate yaw error
    double max_error_on_track = 0.1f; // defines the maximum allowed error between track on current robot pose
    double max_error_yaw = 25.0 * M_PI / 180.0;
    std::vector<std::int64_t> source_ids = {1, 2}; // contains all expected virtual line sensor source ids
    std::vector<std::int64_t> front_source_ids = {1};  // defines front sensors, one will be picked
    std::vector<std::int64_t> rear_source_ids = {2};          // defines rear sensors, one will be picked
    
    struct {
      controller::Pid::Parameter stay_on_line = {
         1.0, 0.0, 0.0, 0.5, 1.0, true};
      controller::Pid::Parameter orientate_to_line = {
         1.0, 0.0, 0.0, M_PI_2, 1.0, true};
    } pid;
  };

  SickLineController();
  ~SickLineController() override = default;

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
  enum class Action {
    NONE = 0,
    STRAIGHT,
    TURN_LEFT,
    TURN_RIGHT,
  };
  enum class Track {
    MIDDLE = 0,
    LEFT,
    RIGHT,
  };

  void callbackLineSensor(const sick_lidar_localization::msg::LineMeasurementMessage0404& msg);
  void callbackAction(const std_msgs::msg::String& msg);
  void processDistances();
  std::size_t getBestIndex(const std::vector<std::int64_t>& group_source_ids);

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> _pub_velocity;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> _pub_on_track;
  std::shared_ptr<rclcpp::Subscription<sick_lidar_localization::msg::LineMeasurementMessage0404>> _sub_line_sensor;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> _sub_action;

  const Parameter _parameter;

  struct {
    std::array<bool, NUM_SENSORS> line_distance_received;
    std::array<double, NUM_SENSORS> line_distance;
    std::array<bool, NUM_SENSORS> valid_line_distance;
    std::array<std::uint8_t, NUM_SENSORS> cnt_lpc;
    std::uint64_t current_telegram;
    std::shared_ptr<controller::ControllerInterface> stay_on_line;
    std::shared_ptr<controller::ControllerInterface> orientate_to_line;
    rclcpp::Time stamp_last_processing;
    std::array<Action, NUM_SENSORS> action = { Action::NONE };
    std::array<Track, NUM_SENSORS> track = { Track::MIDDLE };
  } _processing_data;
};

} // end namespace fleet
} // end namespace eduart
