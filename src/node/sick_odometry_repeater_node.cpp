#include "sick_odometry_repeater_node.hpp"

#include <edu_robot/angle.hpp>

namespace eduart {
namespace fleet {

SickOdometryRepeater::Parameter SickOdometryRepeater::get_parameter(const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  ros_node.declare_parameter<int>("source_id", default_parameter.source_id);

  Parameter parameter = default_parameter;
  parameter.source_id = ros_node.get_parameter("source_id", parameter.source_id);

  return parameter;
}
  
SickOdometryRepeater::SickOdometryRepeater()
  : rclcpp::Node("sick_odometry_repeater")
  , _parameter(get_parameter(Parameter(), *this))
{
  _sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
    "in/odom", rclcpp::QoS(2).best_effort(),
    std::bind(&SickOdometryRepeater::callbackOdometry, this, std::placeholders::_1)
  );

  _pub_odometry = create_publisher<sick_lidar_localization_msgs::msg::OdometryMessage0104>(
    "out/odom", rclcpp::QoS(2).reliable()
  );
}

void SickOdometryRepeater::callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
{
  sick_lidar_localization_msgs::msg::OdometryMessage0104 odom;

  odom.header = msg->header;
  odom.source_id = _parameter.source_id;
  odom.telegram_count = _telegram_counter++;
  odom.timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000 + msg->header.stamp.nanosec / 1000; // in µs

  odom.x_velocity = msg->twist.twist.linear.x * 1000; // in mm/s
  odom.y_velocity = msg->twist.twist.linear.y * 1000; // in mm/s 
  odom.angular_velocity = robot::AnglePiToPi(msg->twist.twist.angular.z).degree() * 1000; // in mdeg/s

  _pub_odometry->publish(odom);
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::SickOdometryRepeater>());
  rclcpp::shutdown();

  return 0;
}
