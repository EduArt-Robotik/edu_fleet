 /**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <edu_robot/angle.hpp>

#include "edu_fleet/action/robot_rotate.hpp"
#include "edu_fleet/controller/pid.hpp"

namespace eduart {
namespace fleet {

class RobotRotateNode : public rclcpp::Node
{
public:
  struct Parameter {
    std::chrono::milliseconds process_interval{50}; // 50ms==20Hz
    robot::AnglePiToPi yaw_error_tolerance{robot::Angle::createFromDegree(3.0)}; // 3 degree
    float min_yaw_rate{0.05f}; // [rad/s]
    float kp{2.5f};
  };

  RobotRotateNode();
  ~RobotRotateNode() override;

private:
  static Parameter get_parameter(const Parameter &default_parameter, rclcpp::Node &ros_node);
  void callbackOdometry(std::shared_ptr<nav_msgs::msg::Odometry> msg);
  rclcpp_action::GoalResponse callbackAcceptGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const edu_fleet::action::RobotRotate::Goal> goal);
  void callbackGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle);
  void executeRotate(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle);

  const Parameter _parameter;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odometry;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_velocity;
  std::shared_ptr<rclcpp_action::Server<edu_fleet::action::RobotRotate>> _action_server;
  std::shared_ptr<fleet::controller::Pid> _pid_controller;

  struct {
    std::mutex mutex;
    robot::AnglePiToPi yaw;
    robot::AnglePiToPi start_yaw;
    robot::AnglePiToPi end_yaw;
    std::atomic_bool is_executing{false};
    std::atomic_bool goal_is_accepted{false};
    float yaw_rate = 0.0f;
  } _data;
};
  
} // end namespace fleet
} // end namespace eduart