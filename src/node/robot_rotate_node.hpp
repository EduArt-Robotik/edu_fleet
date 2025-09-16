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

namespace eduart {
namespace fleet {

class RobotRotateNode : public rclcpp::Node
{
public:
  struct Parameter {
    
  };

  RobotRotateNode();
  ~RobotRotateNode() override;

private:
  void callbackOdometry(std::shared_ptr<nav_msgs::msg::Odometry> msg);
  rclcpp_action::GoalResponse callbackAcceptGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const edu_fleet::action::RobotRotate::Goal> goal);
  void callbackGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle);
  void executeRotate(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle);

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odometry;
  std::shared_ptr<rclcpp_action::Server<edu_fleet::action::RobotRotate>> _action_server;

  struct {
    std::mutex mutex;
    robot::AnglePiToPi yaw;
    robot::AnglePiToPi start_yaw;
    robot::AnglePiToPi end_yaw;
  } _processing;
};
  
} // end namespace fleet
} // end namespace eduart