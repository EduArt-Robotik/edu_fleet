#include "pose_controller_node.hpp"
#include "angle.hpp"

#include <functional>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <memory>
#include <rclcpp/executors.hpp>

namespace eduart {
namespace fleet {

PoseController::Parameter PoseController::get_parameter(rclcpp::Node &ros_node)
{
  PoseController::Parameter parameter;

  ros_node.declare_parameter<double>("pid.kp", parameter.pid.kp);
  ros_node.declare_parameter<double>("pid.ki", parameter.pid.ki);
  ros_node.declare_parameter<double>("pid.kd", parameter.pid.kd);
  ros_node.declare_parameter<bool>("pid.use_anti_windup", parameter.pid.use_anti_windup);
  ros_node.declare_parameter<double>("pid.limit", parameter.pid.limit);

  parameter.pid.kp = ros_node.get_parameter("pid.kp").as_double();
  parameter.pid.ki = ros_node.get_parameter("pid.ki").as_double();
  parameter.pid.kd = ros_node.get_parameter("pid.kd").as_double();
  parameter.pid.use_anti_windup = ros_node.get_parameter("pid.use_anti_windup").as_bool();
  parameter.pid.limit = ros_node.get_parameter("pid.limit").as_double();

  return parameter;
}

PoseController::PoseController()
  : rclcpp::Node("pose_controller")
  , _parameter(get_parameter(*this))
{
  for (auto& controller : _controller) {
    controller.parameter = _parameter.pid;
    controller.reset();
  }

  _controller_set_point = std::make_unique<geometry_msgs::msg::Pose>();
  _controller_set_point->position.x = 0.0;
  _controller_set_point->position.y = 0.0;
  _controller_set_point->position.z = 0.0;
  _controller_set_point->orientation.w = 1.0;
  _controller_set_point->orientation.x = 0.0;
  _controller_set_point->orientation.y = 0.0;
  _controller_set_point->orientation.z = 0.0;

  _controller_output = std::make_unique<geometry_msgs::msg::Twist>();
  _controller_output->linear.x = 0.0;
  _controller_output->linear.y = 0.0;
  _controller_output->linear.z = 0.0;
  _controller_output->angular.x = 0.0;
  _controller_output->angular.y = 0.0;
  _controller_output->angular.z = 0.0;

  _sub_current_pose = create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose_feedback",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&PoseController::callbackCurrentPose, this, std::placeholders::_1)
  );

  _stamp_last_processed = get_clock()->now();
}

PoseController::~PoseController()
{

}

static AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

void PoseController::callbackCurrentPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg)
{
  const auto now = get_clock()->now();
  const double dt = (now - _stamp_last_processed).seconds();

  // Linear X
  _controller_output->linear.x = _controller[0](_controller_set_point->position.x, pose_msg->pose.position.x, dt);

  // Linear Y
  _controller_output->linear.y = _controller[1](_controller_set_point->position.y, pose_msg->pose.position.y, dt);  

  // Yaw Orientation
  const auto q_yaw_feedback = pose_msg->pose.orientation;
  const AnglePiToPi yaw_feedback = quaternion_to_yaw(q_yaw_feedback);
  const AnglePiToPi yaw_set_point = quaternion_to_yaw(_controller_set_point->orientation);

  _controller_output->angular.z = _controller[2](yaw_set_point, yaw_feedback, dt);

  // Publishing Result
  _pub_twist->publish(*_controller_output);
}

} // end namespace fleet
} // end namespace eduart



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<eduart::fleet::PoseController>());
  rclcpp::shutdown();

  return 0;
}