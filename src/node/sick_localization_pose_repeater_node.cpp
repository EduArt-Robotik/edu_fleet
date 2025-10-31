#include "sick_localization_pose_repeater_node.hpp"

#include <edu_robot/angle.hpp>
#include <edu_robot/algorithm/rotation.hpp>

namespace eduart {
namespace fleet {

SickLocalizationPoseRepeater::Parameter SickLocalizationPoseRepeater::get_parameter(
  const Parameter &default_parameter,
  rclcpp::Node &ros_node)
{
  ros_node.declare_parameter<std::string>("tf_map_frame_id", default_parameter.tf_map_frame_id);
  ros_node.declare_parameter<std::string>("tf_robot_frame_id", default_parameter.tf_robot_frame_id);

  std::cout << "default_parameter.tf_map_frame_id: " << default_parameter.tf_map_frame_id << "\n";
  std::cout << "default_parameter.tf_robot_frame_id: " << default_parameter.tf_robot_frame_id << "\n";

  Parameter parameter = default_parameter;

  parameter.tf_map_frame_id = ros_node.get_parameter("tf_map_frame_id").as_string();
  parameter.tf_robot_frame_id = ros_node.get_parameter("tf_robot_frame_id").as_string();

  std::cout << "parameter.tf_map_frame_id: " << parameter.tf_map_frame_id << "\n";
  std::cout << "parameter.tf_robot_frame_id: " << parameter.tf_robot_frame_id << "\n";

  return parameter;
}

SickLocalizationPoseRepeater::SickLocalizationPoseRepeater()
  : rclcpp::Node("sick_localization_pose_repeater")
  ,  _parameter(get_parameter(Parameter(), *this))
  , _tf_broadcaster(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
{
  _sub_odometry = create_subscription<sick_lidar_localization_msgs::msg::LocalizationControllerResultMessage0502>(
    "in/localization", rclcpp::QoS(2).best_effort(),
    std::bind(&SickLocalizationPoseRepeater::callbackOdometry, this, std::placeholders::_1)
  );

  _pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>(
    "out/pose", rclcpp::QoS(2).reliable()
  );
}

void SickLocalizationPoseRepeater::callbackOdometry(
  std::shared_ptr<const sick_lidar_localization_msgs::msg::LocalizationControllerResultMessage0502> msg)
{
  geometry_msgs::msg::PoseStamped pose_out;

  pose_out.header = msg->header;

  // position
  pose_out.pose.position.x = msg->x / 1000.0; // in m
  pose_out.pose.position.y = msg->y / 1000.0; // in m
  pose_out.pose.position.z = 0.0;

  // orientation
  const double yaw = robot::AnglePiToPi::createFromDegree(msg->heading /* in mdeg */ / 1000.0).radian(); // in rad

  pose_out.pose.orientation.x = 0.0;
  pose_out.pose.orientation.y = 0.0;
  pose_out.pose.orientation.z = std::sin(yaw * 0.5);
  pose_out.pose.orientation.w = std::cos(yaw * 0.5);

  _pub_pose->publish(pose_out);

  // broadcast TF
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header = pose_out.header;
  tf_msg.header.frame_id = _parameter.tf_map_frame_id;
  tf_msg.child_frame_id  = _parameter.tf_robot_frame_id;

  // std::cout << "Broadcasting TF from '" << tf_msg.header.frame_id << "' to '" << tf_msg.child_frame_id << "'\n";
  // std::cout << "parameter.tf_map_frame_id: " << std::hex << _parameter.tf_map_frame_id << "\n";
  // std::cout << "parameter.tf_robot_frame_id: " << std::hex << _parameter.tf_robot_frame_id << "\n";

  tf_msg.transform.rotation = pose_out.pose.orientation;
  tf_msg.transform.translation.x = pose_out.pose.position.x;
  tf_msg.transform.translation.y = pose_out.pose.position.y;
  tf_msg.transform.translation.z = pose_out.pose.position.z;

  _tf_broadcaster->sendTransform(tf_msg);
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<eduart::fleet::SickLocalizationPoseRepeater>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
