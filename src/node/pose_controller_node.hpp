 /**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/srv/get_transform.hpp>
#include <edu_fleet/controller/pid.hpp>

#include <edu_robot/angle.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <aruco_opencv_msgs/msg/aruco_detection.hpp>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <rclcpp/timer.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>

#include <memory>
#include <array>

namespace eduart {
namespace fleet {

class PoseController : public rclcpp::Node
{
public:
  struct Parameter {
    controller::Pid::Parameter pid_linear;
    controller::Pid::Parameter pid_angular;

    std::string frame_id = "map"; //> all received messages must be in this frame
  };

  PoseController();
  ~PoseController() override;

  static Parameter get_parameter(rclcpp::Node& ros_node);

private:
  void callbackCurrentPose(std::shared_ptr<const nav_msgs::msg::Odometry> odometry_msg);
  void callbackTargetPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg);
  void process();
  std::string getRobotName() const;

  Parameter _parameter;
  std::array<std::unique_ptr<controller::ControllerInterface>, 3> _controller;
  std::unique_ptr<geometry_msgs::msg::Pose> _set_point;
  std::unique_ptr<geometry_msgs::msg::Pose> _feedback;
  std::unique_ptr<geometry_msgs::msg::Twist> _output;
  rclcpp::Time _stamp_last_processed;

  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_current_pose;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> _sub_target_pose;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;

  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
};

} // end namespace fleet
} // end namespace eduart
