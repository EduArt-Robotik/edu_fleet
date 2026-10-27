#include "triton_pose_repeater_node.hpp"

namespace eduart {
namespace fleet {

TritonPoseRepeater::Parameter TritonPoseRepeater::get_parameter(
  const Parameter & default_parameter,
  rclcpp_lifecycle::LifecycleNode & ros_node)
{
  Parameter parameter = default_parameter;

  // Add parameter retrieval here

  return parameter;
}

TritonPoseRepeater::TritonPoseRepeater()
: rclcpp_lifecycle::LifecycleNode("triton_pose_repeater_node"),
  _parameter(get_parameter(Parameter(), *this))
{
  _pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("out/pose", rclcpp::QoS(2).reliable());
  _sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
    "in/odometry",
    rclcpp::QoS(2).best_effort(),
    std::bind(&TritonPoseRepeater::callbackOdometry, this, std::placeholders::_1)
  );
}

void TritonPoseRepeater::callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
{
  auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_msg->header = msg->header;
  pose_msg->pose = msg->pose.pose;

  _pub_pose->publish(*pose_msg);
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eduart::fleet::TritonPoseRepeater>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
