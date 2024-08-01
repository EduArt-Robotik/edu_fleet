#include "fleet_localization_node.hpp"

#include <edu_fleet/kalman_filter/extended_kalman_filter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>

#include <tf2/transform_storage.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

#include <Eigen/Geometry>

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
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  , _sensor_model_imu(std::make_shared<SensorModelImu>("sensor_model_imu"))
  , _sensor_model_odometry(std::make_shared<SensorModelOdometry>("sensor_model_odometry"))
  , _sensor_model_pose(std::make_shared<SensorModelPose>("sensor_model_pose"))
{
  _robot.resize(_parameter.number_of_robots());

  for (std::size_t i = 0; i < _robot.size(); ++i) {
    // instantiate ROS subscriptions
    // _robot[i].sub_imu = 
    _robot[i].sub_imu = create_subscription<sensor_msgs::msg::Imu>(
      _parameter.robot_name[i] + "/imu",
      rclcpp::QoS(5).best_effort(),
      [this, i](std::shared_ptr<const sensor_msgs::msg::Imu> msg) {
        callbackImu(msg, i);
      }
    );
    _robot[i].sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
      _parameter.robot_name[i] + "/odometry",
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

    // bring up services
    _srv_reset = create_service<std_srvs::srv::Trigger>(
      "reset",
      std::bind(&FleetLocalization::callbackReset, this, std::placeholders::_1, std::placeholders::_2)
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
  publishRobotState(robot_index);
}

void FleetLocalization::callbackOdometry(
  std::shared_ptr<const nav_msgs::msg::Odometry> msg, const std::size_t robot_index)
{
  // \todo check time stamp!
  _sensor_model_odometry->process(msg);
  _robot[robot_index].kalman_filter->process(_sensor_model_odometry);
  publishRobotState(robot_index);
}

void FleetLocalization::callbackPose(
  std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg, const std::size_t robot_index)
{
  // \todo check time stamp!
  _sensor_model_pose->process(msg);
  _robot[robot_index].kalman_filter->process(_sensor_model_pose);
  publishRobotState(robot_index);
}

void FleetLocalization::callbackReset(
  std::shared_ptr<const std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response)
{
  (void)request;
  const std::size_t dimension = FilterModelMecanum::attribute_pack::size();

  for (auto& robot : _robot) {
    robot.kalman_filter->initialize(
      Eigen::Vector<kalman_filter::Data, dimension>::Zero(),
      Eigen::Matrix<kalman_filter::Data, dimension, dimension>::Identity() * 1000.0
    );
  }

  response->success = true;
  response->message = "fleet localization reset";
}

void FleetLocalization::publishRobotState(const std::size_t robot_index)
{
  const auto& kalman_filter = _robot[robot_index].kalman_filter;
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = kalman_filter->stamp();
  transform.header.frame_id = "map";
  transform.child_frame_id = _parameter.robot_name[robot_index] + "/base_footprint";

  // transform
  const auto& state = kalman_filter->state();

  // position
  transform.transform.translation.x = state.x();
  transform.transform.translation.y = state.y();

  // orientation  
  Eigen::Quaterniond orientation(Eigen::AngleAxisd(state.yaw(), Eigen::Vector3d::UnitZ()));
  transform.transform.rotation.w = orientation.w();
  transform.transform.rotation.x = orientation.x();
  transform.transform.rotation.y = orientation.y();
  transform.transform.rotation.z = orientation.z();

  _tf_broadcaster->sendTransform(transform);
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