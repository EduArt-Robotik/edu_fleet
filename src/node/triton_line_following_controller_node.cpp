#include "triton_line_following_controller_node.hpp"

#include <rclcpp/executors.hpp>

#include <edu_robot/algorithm/rotation.hpp>

namespace eduart {
namespace fleet {

TritonLineFollowingController::Parameter TritonLineFollowingController::get_parameter(
  const Parameter& default_parameter, rclcpp_lifecycle::LifecycleNode& ros_node)
{
  ros_node.declare_parameter<double>("pid.y.kp", default_parameter.pid.y.kp);
  ros_node.declare_parameter<double>("pid.y.limit", default_parameter.pid.y.limit);
  ros_node.declare_parameter<double>("pid.heading.kp", default_parameter.pid.heading.kp);
  ros_node.declare_parameter<double>("pid.heading.limit", default_parameter.pid.heading.limit);

  Parameter parameter = default_parameter;

  parameter.pid.y.kp = ros_node.get_parameter("pid.y.kp").as_double();
  parameter.pid.y.limit = ros_node.get_parameter("pid.y.limit").as_double();
  parameter.pid.heading.kp = ros_node.get_parameter("pid.heading.kp").as_double();
  parameter.pid.heading.limit = ros_node.get_parameter("pid.heading.limit").as_double();

  return parameter;
}

TritonLineFollowingController::TritonLineFollowingController()
  : rclcpp_lifecycle::LifecycleNode("triton_line_following_controller")
  , _parameter(get_parameter(Parameter(), *this))
  , _pid_y(std::make_shared<controller::Pid>(_parameter.pid.y))
  , _pid_heading(std::make_shared<controller::Pid>(_parameter.pid.heading))
{

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_configure(
  const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_deactivate(
  const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_cleanup(
  const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_shutdown(
  const rclcpp_lifecycle::State& previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void TritonLineFollowingController::callbackLineFollowingPoses(std::shared_ptr<const geometry_msgs::msg::PoseArray> msg)
{
  RCLCPP_INFO(
    get_logger(), "robot pose x = %f, y = %f, yaw = %f.",
    msg->poses[0].position.x,
    msg->poses[0].position.y,
    robot::algorithm::quaternion_to_yaw(msg->poses[0].orientation).radian()
  );
  RCLCPP_INFO(
    get_logger(), "target pose x = %f, y = %f, yaw = %f.",
    msg->poses[1].position.x,
    msg->poses[1].position.y,
    robot::algorithm::quaternion_to_yaw(msg->poses[1].orientation).radian()
  );

  
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<eduart::fleet::TritonLineFollowingController>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
