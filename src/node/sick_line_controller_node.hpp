 /**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/controller/pid.hpp>

#include <rclcpp/node.hpp>

#include <sick_lidar_localization/msg/line_measurement_message0404.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

namespace eduart {
namespace fleet {

class SickLineController : public rclcpp::Node
{
public:
  struct Parameter {
    double d_x = 0.25f; // distance between line sensors in meter
    // double gain_yaw = 1.0f; // used to compensate yaw error
    double max_error_on_track = 0.1f; // defines the maximum allowed error between track on current robot pose
    double max_error_yaw = 25.0 * M_PI / 180.0;
    std::vector<std::int64_t> source_ids = {1, 2}; // contains all expected virtual line sensor source ids
    
    struct {
      controller::Pid::Parameter stay_on_line = {
         1.0, 0.0, 0.0, 0.5, 1.0, true};
      controller::Pid::Parameter orientate_to_line = {
         1.0, 0.0, 0.0, M_PI_2, 1.0, true};
    } pid;
  };

  SickLineController();
  ~SickLineController() override = default;

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void callbackLineSensor(const sick_lidar_localization::msg::LineMeasurementMessage0404& msg);

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_velocity;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> _pub_on_track;
  std::shared_ptr<rclcpp::Subscription<sick_lidar_localization::msg::LineMeasurementMessage0404>> _sub_line_sensor;

  const Parameter _parameter;

  struct {
    std::array<bool, 2> line_distance_received;
    std::array<double, 2> line_distance;
    std::array<bool, 2> valid_line_distance;
    std::uint64_t current_telegram;
    std::shared_ptr<controller::ControllerInterface> stay_on_line;
    std::shared_ptr<controller::ControllerInterface> orientate_to_line;
    rclcpp::Time stamp_last_processing;
  } _processing_data;
};

} // end namespace fleet
} // end namespace eduart
