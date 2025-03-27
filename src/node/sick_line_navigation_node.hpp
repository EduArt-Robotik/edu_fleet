 /**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/bool.hpp>

#include <edu_robot/msg/set_lighting_color.hpp>
#include <edu_robot/srv/set_mode.hpp>

#include <edu_perception/msg/lidar_field_evaluation.hpp>

#include <sick_lidar_localization/msg/code_measurement_message0304.hpp>

namespace eduart {
namespace fleet {

class SickLineNavigation : public rclcpp::Node
{
public:
  struct Parameter {
    float move_velocity_slow = 0.15f;
    float move_velocity_middle = 0.3f;
    float move_velocity_fast = 0.5f;
    float stop_time = 5.0f;
  };

  SickLineNavigation();
  ~SickLineNavigation() override = default;

  static Parameter get_parameter(const Parameter &default_parameter, rclcpp::Node &ros_node);

private:
  void callbackOnTrack(std::shared_ptr<const std_msgs::msg::Bool> msg);
  void callbackCode(std::shared_ptr<const sick_lidar_localization::msg::CodeMeasurementMessage0304> msg);
  void callbackFieldEvaluation(std::shared_ptr<const edu_perception::msg::LidarFieldEvaluation> msg);
  void deactivateStop();

  void process();

  const Parameter _parameter;

  struct {
    bool on_track = false;
    bool warnfeld_active = false;
    bool schutzfeld_active = false;
    bool stop_active = false;
    float requested_velocity = 0.0;
  } _processing_data;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_velocity;
  std::shared_ptr<rclcpp::Publisher<edu_robot::msg::SetLightingColor>> _pub_lighting_color;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> _sub_on_track;
  std::shared_ptr<rclcpp::Subscription<sick_lidar_localization::msg::CodeMeasurementMessage0304>> _sub_code;
  std::shared_ptr<rclcpp::Subscription<edu_perception::msg::LidarFieldEvaluation>> _sub_field_evaluation;
  std::shared_ptr<rclcpp::Client<edu_robot::srv::SetMode>> _client_set_mode;
  std::shared_ptr<rclcpp::TimerBase> _timer_processing;
  std::shared_ptr<rclcpp::TimerBase> _timer_process_stopping;
};

} // end namespace fleet
} // end namespace eduart
