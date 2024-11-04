#include "line_controller_node.hpp"

#include <rclcpp/executors.hpp>

namespace eduart {
namespace fleet {

LineController::Parameter LineController::get_parameter(const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  (void)ros_node;
  return default_parameter;
}

LineController::LineController()
  : rclcpp::Node("line_controller")
  , _parameter(get_parameter({}, *this))
{
  // Configuring ROS Topics and Services
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/velocity", rclcpp::QoS(2).reliable()
  );
  _sub_line_sensor = create_subscription<sick_lidar_localization::msg::LineMeasurementMessage0404>(
    "in/line_detection",
    rclcpp::QoS(10).reliable(),
    std::bind(&LineController::callbackLineSensor, this, std::placeholders::_1)
  );

  // Initializing Processing Data
  _processing_data.line_distance_received.fill(false);
  _processing_data.line_distance.fill(0.0);
  _processing_data.current_telegram = 0;

  _processing_data.stay_on_line = std::make_shared<controller::Pid>(_parameter.pid.stay_on_line);
  _processing_data.orientate_to_line = std::make_shared<controller::Pid>(_parameter.pid.orientate_to_line);
  _processing_data.stay_on_line->reset();
  _processing_data.orientate_to_line->reset();
  _processing_data.stamp_last_processing = get_clock()->now();
}

void LineController::callbackLineSensor(const sick_lidar_localization::msg::LineMeasurementMessage0404& msg)
{
  // Start new measurement cycle if telegram number changed.
  if (msg.telegram_count != _processing_data.current_telegram) {
    RCLCPP_INFO(get_logger(), "start new line measurement set.");
    _processing_data.current_telegram = msg.telegram_count;
    _processing_data.line_distance_received.fill(false);
    _processing_data.line_distance.fill(0.0);
  }

  // Assign line measurement.
  for (std::size_t i = 0; i < _parameter.source_ids.size(); ++i) {
    // First get correct index i of received line measurement.
    if (msg.source_id == _parameter.source_ids[i]) {
      // Found correct index i.
      // Use lcp2 only at the moment (middle line)
      // std::cout << "source id = " << msg.source_id << std::endl;
      // std::cout << "found index = " << i << std::endl;
      _processing_data.line_distance[i] = -msg.lcp2 / 1000.0f; // convert into meter
      // std::cout << "assigned distance = " << _processing_data.line_distance[i] << std::endl;
      _processing_data.line_distance_received[i] = true;
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

  // Line measurements are finished.
  RCLCPP_INFO(get_logger(), "line measurements complete.");
  // Get dt
  const auto stamp_now = get_clock()->now();
  const double dt = std::min(0.1, (stamp_now - _processing_data.stamp_last_processing).seconds());

  // Calculate Yaw error.
  const double error_line = _processing_data.line_distance[0] - _processing_data.line_distance[1];
  const double error_yaw = std::atan2(error_line, _parameter.d_x);

  const double yaw_rate = _processing_data.orientate_to_line->process(0.0, error_yaw, dt);

  RCLCPP_INFO(get_logger(), "yaw error = %f.", error_yaw);
  RCLCPP_INFO(get_logger(), "yaw rate output = %f.", yaw_rate);

  // Calculate error in y direction.
  const double error_y = error_line / 2.0f + _processing_data.line_distance[1];

  const double vel_y = _processing_data.stay_on_line->process(0.0, error_y, dt);

  RCLCPP_INFO(get_logger(), "error in y direction = %f.", error_y);
  RCLCPP_INFO(get_logger(), "velocity y = %f.", vel_y);

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
  rclcpp::spin(std::make_shared<eduart::fleet::LineController>());
  rclcpp::shutdown();

  return 0;
}
