#include "fleet_localization_node.hpp"

#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>

namespace eduart {
namespace fleet {

FleetLocalization::Parameter FleetLocalization::get_parameter(
  const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<int>(name + ".number_of_robots", default_parameter.number_of_robots());

  const std::size_t number_of_robots = ros_node.get_parameter(name + ".number_of_robots").as_int();

  parameter.robot_name.resize(number_of_robots);
  return parameter;
}

FleetLocalization::FleetLocalization(const Parameter& parameter)
  : rclcpp::Node("fleet_localization")
  , _parameter(parameter)
{
  _robot.resize(_parameter.number_of_robots());

  for (std::size_t i = 0; i < _robot.size(); ++i) {
    // instantiate ROS subscriptions
    _robot[i].sub_imu = create_subscription<sensor_msgs::msg::Imu>(
      _parameter.robot_name[i] + "/imu",
      rclcpp::QoS(5).best_effort(),
      std::bind(&FleetLocalization::callbackImu, this, std::placeholders::_1, i)
    );
    _robot[i].sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
      _parameter.robot_name[i] + "/odom",
      rclcpp::QoS(5).best_effort(),
      std::bind(&FleetLocalization::callbackOdometry, this, std::placeholders::_1, i)
    );
    _robot[i].sub_pose = create_subscription<geometry_msgs::msg::PoseStamped>(
      _parameter.robot_name[i] + "/marker_pose",
      rclcpp::QoS(5).best_effort(),
      std::bind(&FleetLocalization::callbackPose, this, std::placeholders::_1, i)
    );

    // instantiate Kalman filter
    
  }
}

FleetLocalization::~FleetLocalization()
{

}



} // end namespace fleet
} // end namespace eduart
