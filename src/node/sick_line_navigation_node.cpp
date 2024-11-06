#include "sick_line_navigation_node.hpp"

namespace eduart {
namespace fleet {

using namespace std::chrono_literals;

SickLineNavigation::Parameter SickLineNavigation::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter("move_velocity", default_parameter.move_velocity);

  parameter.move_velocity = ros_node.get_parameter("move_velocity").as_double();

  return default_parameter;
}

SickLineNavigation::SickLineNavigation()
  : rclcpp::Node("sick_line_navigation")
  , _parameter(get_parameter({}, *this))
  , _on_track(false)
{
  // ROS Related
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/cmd_vel",
    rclcpp::QoS(10).reliable()
  );
  _sub_on_track = create_subscription<std_msgs::msg::Bool>(
    "in/on_track", 
    rclcpp::QoS(2).transient_local(), 
    std::bind(&SickLineNavigation::callbackOnTrack, this, std::placeholders::_1)
  );
  _sub_code = create_subscription<sick_lidar_localization::msg::CodeMeasurementMessage0304>(
    "in/code", 
    rclcpp::QoS(10).reliable(), 
    std::bind(&SickLineNavigation::callbackCode, this, std::placeholders::_1)
  );
  _timer_processing = create_timer(100ms, std::bind(&SickLineNavigation::process, this));
}

void SickLineNavigation::callbackOnTrack(std::shared_ptr<const std_msgs::msg::Bool> msg)
{
  RCLCPP_INFO(get_logger(), "received on track flag: %i.", msg->data);
  _on_track = msg->data;
}

void SickLineNavigation::callbackCode(std::shared_ptr<const sick_lidar_localization::msg::CodeMeasurementMessage0304> msg)
{
  RCLCPP_INFO(get_logger(), "received code \"%i\".", msg->code);
}

void SickLineNavigation::process()
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x = _on_track ? _parameter.move_velocity : 0.0;
  
  _pub_velocity->publish(twist);
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::SickLineNavigation>());
  rclcpp::shutdown();

  return 0;
}