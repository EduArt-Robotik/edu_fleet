#include "sick_line_controller_node.hpp"

#include <rclcpp/executors.hpp>

namespace eduart {
namespace fleet {

SickLineController::Parameter SickLineController::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  ros_node.declare_parameter<double>("d_x", default_parameter.d_x);
  ros_node.declare_parameter<double>("max_error_on_track", default_parameter.max_error_on_track);
  ros_node.declare_parameter<double>("max_error_yaw", default_parameter.max_error_yaw);
  ros_node.declare_parameter<std::vector<std::int64_t>>("source_ids", default_parameter.source_ids);

  ros_node.declare_parameter<double>("pid.linear.kp", default_parameter.pid.stay_on_line.kp);
  ros_node.declare_parameter<double>("pid.linear.limit", default_parameter.pid.stay_on_line.limit);
  ros_node.declare_parameter<double>("pid.angular.kp", default_parameter.pid.orientate_to_line.kp);
  ros_node.declare_parameter<double>("pid.angular.limit", default_parameter.pid.orientate_to_line.limit);

  Parameter parameter;

  parameter.d_x = ros_node.get_parameter("d_x").as_double();
  parameter.max_error_on_track = ros_node.get_parameter("max_error_on_track").as_double();
  parameter.max_error_yaw = ros_node.get_parameter("max_error_yaw").as_double();
  parameter.source_ids = ros_node.get_parameter("source_ids").as_integer_array();

  parameter.pid.stay_on_line.kp = ros_node.get_parameter("pid.linear.kp").as_double();
  parameter.pid.stay_on_line.limit = ros_node.get_parameter("pid.linear.limit").as_double();
  parameter.pid.orientate_to_line.kp = ros_node.get_parameter("pid.angular.kp").as_double();
  parameter.pid.orientate_to_line.limit = ros_node.get_parameter("pid.angular.limit").as_double();

  return parameter;
}

SickLineController::SickLineController()
  : rclcpp::Node("line_controller")
  , _parameter(get_parameter({}, *this))
{
  // Configuring ROS Topics and Services
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/velocity", rclcpp::QoS(2).reliable()
  );
  _pub_on_track = create_publisher<std_msgs::msg::Bool>(
    "out/on_track", 
    rclcpp::QoS(2).transient_local()
  );
  _sub_line_sensor = create_subscription<sick_lidar_localization::msg::LineMeasurementMessage0404>(
    "in/line_detection",
    rclcpp::QoS(10).reliable(),
    std::bind(&SickLineController::callbackLineSensor, this, std::placeholders::_1)
  );

  // Initializing Processing Data
  _processing_data.line_distance_received.fill(false);
  _processing_data.line_distance.fill(0.0);
  _processing_data.valid_line_distance.fill(false);
  _processing_data.current_telegram = 0;

  _processing_data.stay_on_line = std::make_shared<controller::Pid>(_parameter.pid.stay_on_line);
  _processing_data.orientate_to_line = std::make_shared<controller::Pid>(_parameter.pid.orientate_to_line);
  _processing_data.stay_on_line->reset();
  _processing_data.orientate_to_line->reset();
  _processing_data.stamp_last_processing = get_clock()->now();
}

void SickLineController::callbackLineSensor(const sick_lidar_localization::msg::LineMeasurementMessage0404& msg)
{
  // Start new measurement cycle if telegram number changed.
  if (msg.telegram_count != _processing_data.current_telegram) {
    RCLCPP_INFO(get_logger(), "start new line measurement set.");
    _processing_data.current_telegram = msg.telegram_count;
    _processing_data.line_distance_received.fill(false);
    _processing_data.line_distance.fill(0.0);
    _processing_data.valid_line_distance.fill(false);
  }

  // Assign line measurement.
  for (std::size_t i = 0; i < _parameter.source_ids.size(); ++i) {
    // First get correct index i of received line measurement.
    if (msg.source_id == _parameter.source_ids[i]) {
      // Found correct index i.
      _processing_data.line_distance_received[i] = true;

      if ((msg.cnt_lpc & (1 << 1)) == false) {
        // No valid middle line.
        _processing_data.valid_line_distance[i] = false;
        break;
      }

      // Valid middle line.
      _processing_data.valid_line_distance[i] = true;
      // Use lcp2 only at the moment (middle line)
      _processing_data.line_distance[i] = -msg.lcp2 / 1000.0f; // convert into meter
      break;
    }
  }

  // Check if all needed measurements are received.
  for (const auto received : _processing_data.line_distance_received) {
    if (received == false) {
      // Minium one measurement is missing --> return
      return;
    }
  }
  //else: Line measurements are finished.

  // Check if all measurements are valid.
  for (const auto valid : _processing_data.valid_line_distance) {
    if (valid == false) {
      RCLCPP_ERROR(get_logger(), "Minium one measurement is not valid --> stop robot");

      // Publish command to stop the robot.
      _pub_on_track->publish(std_msgs::msg::Bool());
      _pub_velocity->publish(geometry_msgs::msg::Twist());
      return;
    }
  }
  //else: valid measurement.
  RCLCPP_INFO(get_logger(), "line measurements complete.");
  // Get dt
  const auto stamp_now = get_clock()->now();
  const double dt = std::min(0.1, (stamp_now - _processing_data.stamp_last_processing).seconds());

  // Calculate Yaw error.
  const double error_line = _processing_data.line_distance[0] - _processing_data.line_distance[1];
  const double error_yaw = std::atan2(error_line, _parameter.d_x);

  const double yaw_rate = _processing_data.orientate_to_line->process(0.0, -error_yaw, dt);

  RCLCPP_INFO(get_logger(), "yaw error = %f.", error_yaw);
  RCLCPP_INFO(get_logger(), "yaw rate output = %f.", yaw_rate);

  // Calculate error in y direction.
  const double error_y = error_line / 2.0f + _processing_data.line_distance[1];

  const double vel_y = _processing_data.stay_on_line->process(0.0, -error_y, dt);

  RCLCPP_INFO(get_logger(), "error in y direction = %f.", error_y);
  RCLCPP_INFO(get_logger(), "velocity y = %f.", vel_y);

  // Estimate if robot is on track.
  std_msgs::msg::Bool on_track;

  on_track.data =
    std::abs(error_y) < _parameter.max_error_on_track && std::abs(error_yaw) < _parameter.max_error_yaw;
  _pub_on_track->publish(on_track);

  // Finish processing.
  _processing_data.stamp_last_processing = stamp_now;

  geometry_msgs::msg::Twist twist_out;

  twist_out.linear.y = vel_y;
  twist_out.angular.z = yaw_rate;

  _pub_velocity->publish(twist_out);
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::SickLineController>());
  rclcpp::shutdown();

  return 0;
}
