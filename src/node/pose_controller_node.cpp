#include "pose_controller_node.hpp"
#include "angle.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/executors.hpp>

#include <functional>

namespace eduart {
namespace fleet {

PoseController::Parameter PoseController::get_parameter(rclcpp::Node &ros_node)
{
  PoseController::Parameter parameter;

  ros_node.declare_parameter<double>("pid.linear.kp", parameter.pid_linear.kp);
  ros_node.declare_parameter<double>("pid.linear.ki", parameter.pid_linear.ki);
  ros_node.declare_parameter<double>("pid.linear.kd", parameter.pid_linear.kd);
  ros_node.declare_parameter<bool>("pid.linear.use_anti_windup", parameter.pid_linear.use_anti_windup);
  ros_node.declare_parameter<double>("pid.linear.limit", parameter.pid_linear.limit);
  ros_node.declare_parameter<double>("pid.angular.kp", parameter.pid_angular.kp);
  ros_node.declare_parameter<double>("pid.angular.ki", parameter.pid_angular.ki);
  ros_node.declare_parameter<double>("pid.angular.kd", parameter.pid_angular.kd);
  ros_node.declare_parameter<bool>("pid.angular.use_anti_windup", parameter.pid_angular.use_anti_windup);
  ros_node.declare_parameter<double>("pid.angular.limit", parameter.pid_angular.limit);

  ros_node.declare_parameter<std::string>("frame_robot", parameter.frame_robot);

  ros_node.declare_parameter<double>("set_point.x", parameter.set_point.x);
  ros_node.declare_parameter<double>("set_point.y", parameter.set_point.y);
  ros_node.declare_parameter<double>("set_point.yaw", parameter.set_point.yaw);

  parameter.pid_linear.kp = ros_node.get_parameter("pid.linear.kp").as_double();
  parameter.pid_linear.ki = ros_node.get_parameter("pid.linear.ki").as_double();
  parameter.pid_linear.kd = ros_node.get_parameter("pid.linear.kd").as_double();
  parameter.pid_linear.use_anti_windup = ros_node.get_parameter("pid.linear.use_anti_windup").as_bool();
  parameter.pid_linear.limit = ros_node.get_parameter("pid.linear.limit").as_double();
  parameter.pid_angular.kp = ros_node.get_parameter("pid.angular.kp").as_double();
  parameter.pid_angular.ki = ros_node.get_parameter("pid.angular.ki").as_double();
  parameter.pid_angular.kd = ros_node.get_parameter("pid.angular.kd").as_double();
  parameter.pid_angular.use_anti_windup = ros_node.get_parameter("pid.angular.use_anti_windup").as_bool();
  parameter.pid_angular.limit = ros_node.get_parameter("pid.angular.limit").as_double();
  parameter.frame_robot = ros_node.get_parameter("frame_robot").as_string();

  parameter.set_point.x = ros_node.get_parameter("set_point.x").as_double();
  parameter.set_point.y = ros_node.get_parameter("set_point.y").as_double();
  parameter.set_point.yaw = ros_node.get_parameter("set_point.yaw").as_double();

  return parameter;
}

PoseController::PoseController()
  : rclcpp::Node("pose_controller")
  , _parameter(get_parameter(*this))
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  _controller[0].parameter = _parameter.pid_linear;
  _controller[1].parameter = _parameter.pid_linear;
  _controller[2].parameter = _parameter.pid_angular;

  _controller_set_point = std::make_unique<geometry_msgs::msg::Pose>();
  _controller_set_point->position.x = _parameter.set_point.x;
  _controller_set_point->position.y = _parameter.set_point.y;
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
    rclcpp::QoS(1).best_effort(),
    std::bind(&PoseController::callbackCurrentPose, this, std::placeholders::_1)
  );
  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "twist_output", rclcpp::QoS(1).reliable()
  );
  _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  _stamp_last_processed = get_clock()->now();
  for (auto& controller : _controller) {
    controller.reset();
  }
}

PoseController::~PoseController()
{

}

static AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

static geometry_msgs::msg::Pose transform_pose(const geometry_msgs::msg::Pose& pose_in, const geometry_msgs::msg::Transform& transform)
{
  const Eigen::Quaternionf rot_transform(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  const Eigen::Quaternionf rot_pose(pose_in.orientation.w, pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z);
  const Eigen::Quaternionf new_rot = rot_pose * rot_transform.inverse();
  const Eigen::Vector3f translation = new_rot * Eigen::Vector3f(-transform.translation.x, -transform.translation.y, -transform.translation.z);

  geometry_msgs::msg::Pose pose_out;
  pose_out.position.x = pose_in.position.x + translation.x();
  pose_out.position.y = pose_in.position.y + translation.y();
  pose_out.position.z = pose_in.position.z + translation.z();
  pose_out.orientation.w = new_rot.w();
  pose_out.orientation.x = new_rot.x();
  pose_out.orientation.y = new_rot.y();
  pose_out.orientation.z = new_rot.z();

  return pose_out;
}

void PoseController::callbackCurrentPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg)
{
  // Bring pose in robot coordinate system.
  geometry_msgs::msg::TransformStamped t_qr_code_to_base_link;
  geometry_msgs::msg::PoseStamped pose_in_eduard_red_base_link;
  geometry_msgs::msg::PoseStamped pose_in_base_link;

  try {
    t_qr_code_to_base_link = _tf_buffer->lookupTransform(
       "eduard/red/base_link", "eduard/red/qr_code/rear", tf2::TimePointZero
    );
    pose_in_eduard_red_base_link.pose = transform_pose(pose_msg->pose, t_qr_code_to_base_link.transform);
    pose_in_eduard_red_base_link.header = pose_msg->header;
    pose_in_base_link = _tf_buffer->transform(pose_in_eduard_red_base_link, _parameter.frame_robot);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not transform %s to %s: %s",
      _parameter.frame_robot.c_str(),
      pose_msg->header.frame_id.c_str(),
      ex.what()
    );
    return;
  }

  // For Debugging
  geometry_msgs::msg::TransformStamped qr_code_transform;
  qr_code_transform.header.frame_id = pose_in_base_link.header.frame_id;
  qr_code_transform.header.stamp = get_clock()->now();
  qr_code_transform.child_frame_id = "eduard_red";
  qr_code_transform.transform.translation.x = pose_in_base_link.pose.position.x;
  qr_code_transform.transform.translation.y = pose_in_base_link.pose.position.y;
  qr_code_transform.transform.translation.z = pose_in_base_link.pose.position.z;
  qr_code_transform.transform.rotation = pose_in_base_link.pose.orientation;
  _tf_broadcaster->sendTransform(qr_code_transform);

  // Respect controller set point in pose measurement. (sensor coordinate system will be rotated by the controller output)
  const Eigen::Vector2d position_in_base_link(pose_in_base_link.pose.position.x, pose_in_base_link.pose.position.y);
  const Eigen::Rotation2Dd R_set_point(-_parameter.set_point.yaw);
  const Eigen::Vector2d position_corrected = /* R_set_point * */ position_in_base_link;

  // Process pose controller on each dimension.
  const auto now = get_clock()->now();
  const double dt = (now - _stamp_last_processed).seconds();

  // Linear X
  // _controller_output->linear.x = _controller[0](_controller_set_point->position.x, pose_in_base_link.pose.position.x, dt);
  _controller_output->linear.x = _controller[0](_parameter.set_point.x, position_corrected.x(), dt);

  // Linear Y
  // _controller_output->linear.y = _controller[1](_controller_set_point->position.y, pose_in_base_link.pose.position.y, dt);
  _controller_output->linear.y = _controller[1](_parameter.set_point.y, position_corrected.y(), dt);    

  // Yaw Orientation
  const auto q_yaw_feedback = pose_in_base_link.pose.orientation;
  const AnglePiToPi yaw_feedback = quaternion_to_yaw(q_yaw_feedback);
  // const AnglePiToPi yaw_set_point = quaternion_to_yaw(_controller_set_point->orientation);
  const AnglePiToPi yaw_set_point = _parameter.set_point.yaw;  

  _controller_output->angular.z = AnglePiToPi(_controller[2](yaw_set_point, yaw_feedback, dt));

  // Publishing Result
  _pub_twist->publish(*_controller_output);
  _stamp_last_processed = now;
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