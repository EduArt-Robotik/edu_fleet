#include "fleet_localization_node.hpp"
#include "edu_fleet/kalman_filter/extended_kalman_filter.hpp"
#include "edu_fleet/kalman_filter/filter_model_mecanum.hpp"

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

#include <cstddef>
#include <memory>

namespace eduart {
namespace fleet {

using kalman_filter::FilterModelMecanum;
using kalman_filter::ExtendedKalmanFilter;

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
  , _sensor_model_imu(std::make_shared<SensorModelImu>("sensor_model_imu"))
  , _sensor_model_odometry(std::make_shared<SensorModelOdometry>("sensor_model_odometry"))
  , _sensor_model_pose(std::make_shared<SensorModelPose>("sensor_model_pose"))
{
  _robot.resize(_parameter.number_of_robots());

  for (std::size_t i = 0; i < _robot.size(); ++i) {
    // instantiate ROS subscriptions
    // _robot[i].sub_imu = 
    create_subscription<sensor_msgs::msg::Imu>(
      _parameter.robot_name[i] + "/imu",
      rclcpp::QoS(5).best_effort(),
      [this, i](std::shared_ptr<const sensor_msgs::msg::Imu> msg) {
        callbackImu(msg, i);
      }
    );
    _robot[i].sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
      _parameter.robot_name[i] + "/odom",
      rclcpp::QoS(5).best_effort(),
      [this, i](std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
        callbackOdometry(msg, i);
      }
    );
    _robot[i].sub_pose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      _parameter.robot_name[i] + "/pose",
      rclcpp::QoS(5).best_effort(),
      [this, i](std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg) {
        callbackPose(msg, i);
      }
    );

    // instantiate Kalman filter
    auto filter_model = std::make_unique<FilterModelMecanum>(_parameter.filter_parameter[i]);
    _robot[i].kalman_filter = std::make_unique<ExtendedKalmanFilter<FilterModelMecanum::attribute_pack>>(
      std::move(filter_model));
  }
}

FleetLocalization::~FleetLocalization()
{

}

void FleetLocalization::callbackImu(
  std::shared_ptr<const sensor_msgs::msg::Imu> msg, const std::size_t robot_index)
{
  // \todo check time stamp!
  _sensor_model_imu->process(msg);
  _robot[robot_index].kalman_filter->process(_sensor_model_imu);
}

void FleetLocalization::callbackOdometry(
  std::shared_ptr<const nav_msgs::msg::Odometry> msg, const std::size_t robot_index)
{
  // \todo check time stamp!
  _sensor_model_odometry->process(msg);
  _robot[robot_index].kalman_filter->process(_sensor_model_odometry);
}

void FleetLocalization::callbackPose(
  std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg, const std::size_t robot_index)
{
  // \todo check time stamp!
  _sensor_model_pose->process(msg);
  _robot[robot_index].kalman_filter->process(_sensor_model_pose);
}

} // end namespace fleet
} // end namespace eduart


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<eduart::fleet::FleetLocalization>(eduart::fleet::FleetLocalization::Parameter{}));
  rclcpp::shutdown();

  return 0;
}