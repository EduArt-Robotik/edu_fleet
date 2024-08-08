/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_pack.hpp>
#include <edu_fleet/kalman_filter/extended_kalman_filter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>
#include <edu_fleet/sensor_model/sensor_model_ros.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/duration.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace eduart {
namespace fleet {

using kalman_filter::FilterModelMecanum;
using kalman_filter::ExtendedKalmanFilter;
using kalman_filter::Attribute;
using kalman_filter::AttributePack;
using sensor_model::SensorModelRos;

class RobotLocalization : public rclcpp::Node
{
public:
  struct Parameter {
    std::string robot_name = "eduard/blue";
    kalman_filter::FilterModelMecanum::Parameter filter_parameter;
    rclcpp::Duration input_delay = rclcpp::Duration::from_seconds(0.150); // 150 ms
    rclcpp::Duration output_interval = rclcpp::Duration::from_seconds(0.01); // 10 ms == 100 Hz

    struct {
      struct {
        struct {
          // odometry
          struct {
            double linear = 0.5;
            double angular = 0.124992371;
          } odometry;
          // imu
          struct {
            double linear = 0.2;
            double angular = 5.0 * M_PI / 180.0; // 5°
          } imu;
        } min;
        struct {
          double odometry = 1.0;
        } max;
      } std_dev;
    } limit;
  };

  RobotLocalization(const Parameter& parameter);
  ~RobotLocalization() override;

  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  // methods
  void callbackImu(std::shared_ptr<const sensor_msgs::msg::Imu> msg);
  void callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg);
  void callbackPose(std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg);
  void callbackReset(
    std::shared_ptr<const std_srvs::srv::Trigger_Request> request,
    std::shared_ptr<std_srvs::srv::Trigger_Response> response);
  void publishRobotState();
  void checkIfStateShouldPredicted();
  std::string getFrameIdPrefix() const;  

  // members
  const Parameter _parameter;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> _srv_reset;
  std::shared_ptr<rclcpp::TimerBase> _timer_publish_robot_state;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> _sub_imu;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odometry;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> _sub_pose;

  std::unique_ptr<ExtendedKalmanFilter<FilterModelMecanum::attribute_pack>> _kalman_filter;
  rclcpp::Time _stamp_last_published;

  // sensor models
  using SensorModelImu = SensorModelRos<AttributePack<Attribute::ACC_X, Attribute::ACC_Y, Attribute::YAW_RATE>, sensor_msgs::msg::Imu>;
  using SensorModelOdometry = SensorModelRos<AttributePack<Attribute::VEL_X, Attribute::VEL_Y, Attribute::YAW_RATE>, nav_msgs::msg::Odometry>;
  using SensorModelPose = SensorModelRos<AttributePack<Attribute::W_POS_X, Attribute::W_POS_Y, Attribute::W_YAW>, geometry_msgs::msg::PoseWithCovarianceStamped>;

  std::shared_ptr<SensorModelImu> _sensor_model_imu;
  std::shared_ptr<SensorModelOdometry> _sensor_model_odometry;
  std::shared_ptr<SensorModelPose> _sensor_model_pose;
};

} // end namespace fleet
} // end namespace eduart
