#include "sick_localization_pose_repeater_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <edu_robot/angle.hpp>
#include <edu_robot/algorithm/rotation.hpp>

namespace eduart {
namespace fleet {

SickLocalizationPoseRepeater::Parameter SickLocalizationPoseRepeater::get_parameter(
  const Parameter &default_parameter, rclcpp_lifecycle::LifecycleNode &ros_node)
{
  ros_node.declare_parameter<std::string>("tf_map_frame_id", default_parameter.tf_map_frame_id);
  ros_node.declare_parameter<std::string>("tf_robot_frame_id", default_parameter.tf_robot_frame_id);
  ros_node.declare_parameter<std::string>("tf_target_frame_id", default_parameter.tf_target_frame_id);

  Parameter parameter = default_parameter;

  parameter.tf_map_frame_id = ros_node.get_parameter("tf_map_frame_id").as_string();
  parameter.tf_robot_frame_id = ros_node.get_parameter("tf_robot_frame_id").as_string();
  parameter.tf_target_frame_id = ros_node.get_parameter("tf_target_frame_id").as_string();

  return parameter;
}

SickLocalizationPoseRepeater::SickLocalizationPoseRepeater()
  : rclcpp_lifecycle::LifecycleNode("sick_localization_pose_repeater")
  ,  _parameter(get_parameter(Parameter(), *this))
  , _tf_broadcaster(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
  , _tf_buffer(std::make_shared<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_shared<tf2_ros::TransformListener>(*_tf_buffer))
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
  // get transform for transformation into target frame
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = _tf_buffer->lookupTransform(
      _parameter.tf_target_frame_id, _parameter.tf_robot_frame_id, msg->header.stamp
    );
  }
  catch (const tf2::TransformException& ex) {
    // no transform available --> no valid data to publish
    RCLCPP_ERROR(
      get_logger(), "no transformation available form \"%s\" to \"%s\" --> no data will be published.",
      _parameter.tf_robot_frame_id.c_str(), _parameter.tf_target_frame_id.c_str()
    );
    return;
  }

  // converting data from Sick Lidar Loc and transform it into target frame
  geometry_msgs::msg::PoseStamped pose_out;

  pose_out.header = msg->header;
  pose_out.header.frame_id = _parameter.tf_map_frame_id;

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

  tf2::doTransform(pose_out, pose_out, transform);

  _pub_pose->publish(pose_out);

  // broadcast TF
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header = pose_out.header;
  tf_msg.header.frame_id = _parameter.tf_map_frame_id;
  tf_msg.child_frame_id  = _parameter.tf_robot_frame_id;

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

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
