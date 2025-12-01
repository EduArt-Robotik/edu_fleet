 /**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sick_lidar_localization_msgs/msg/localization_controller_result_message0502.hpp>

#include <tf2_ros/transform_broadcaster.h>

namespace eduart {
namespace fleet {

class SickLocalizationPoseRepeater : public rclcpp_lifecycle::LifecycleNode
{
public:
  struct Parameter {
    std::string tf_map_frame_id = "map";
    std::string tf_robot_frame_id = "base_footprint";
  };

  SickLocalizationPoseRepeater();
  ~SickLocalizationPoseRepeater() override = default;

  static Parameter get_parameter(const Parameter &default_parameter, rclcpp_lifecycle::LifecycleNode &ros_node);

private:
  void callbackOdometry(
    std::shared_ptr<const sick_lidar_localization_msgs::msg::LocalizationControllerResultMessage0502> msg);

  const Parameter _parameter;

  std::shared_ptr<rclcpp::Subscription<sick_lidar_localization_msgs::msg::LocalizationControllerResultMessage0502>> _sub_odometry;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> _pub_pose;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
};

} // end namespace fleet
} // end namespace eduart
