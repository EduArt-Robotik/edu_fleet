 /**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/srv/get_transform.hpp>

#include <edu_robot/angle.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

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

#include "pid_controller.hpp"

namespace eduart {
namespace fleet {

class PoseController : public rclcpp::Node
{
public:
  struct Parameter {
    PidController::Parameter pid_linear;
    PidController::Parameter pid_angular;
    std::string frame_robot = "base_link";
    std::string reference_robot_name = "eduard/blue";
    std::string reference_robot_frame = "eduard/blue/base_link";
    std::string marker_frame = "eduard/blue/qr_code/rear";
    
    struct SetPoint {
      double x   = 0.0;
      double y   = 0.0;
      double yaw = 0.0;
    } set_point;
  };

  PoseController();
  ~PoseController() override;

  static Parameter get_parameter(rclcpp::Node& ros_node);

private:
  void callbackCurrentPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg);
  void callbackMarkerDetection(std::shared_ptr<const aruco_opencv_msgs::msg::ArucoDetection> marker_msg);
  void process(const geometry_msgs::msg::PoseStamped& pose);
  void getTransform();
  std::string getRobotName() const;

  Parameter _parameter;
  std::array<PidController, 3> _controller;
  std::unique_ptr<geometry_msgs::msg::Pose> _controller_set_point;
  std::unique_ptr<geometry_msgs::msg::Twist> _controller_output;
  rclcpp::Time _stamp_last_processed;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> _sub_current_pose;
  std::shared_ptr<rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>> _sub_aruco_marker_detection;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_twist;
  std::shared_ptr<rclcpp::Client<edu_fleet::srv::GetTransform>> _srv_client_get_transform;
  std::shared_ptr<rclcpp::TimerBase> _timer_get_transform;

  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
};

} // end namespace fleet
} // end namespace eduart
