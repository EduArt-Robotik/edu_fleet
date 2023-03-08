#include "pose_controller_node.hpp"
#include "angle.hpp"

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

  ros_node.declare_parameter<double>("pid.kp", parameter.pid.kp);
  ros_node.declare_parameter<double>("pid.ki", parameter.pid.ki);
  ros_node.declare_parameter<double>("pid.kd", parameter.pid.kd);
  ros_node.declare_parameter<bool>("pid.use_anti_windup", parameter.pid.use_anti_windup);
  ros_node.declare_parameter<double>("pid.limit", parameter.pid.limit);
  ros_node.declare_parameter<std::string>("frame_robot", parameter.frame_robot);

  parameter.pid.kp = ros_node.get_parameter("pid.kp").as_double();
  parameter.pid.ki = ros_node.get_parameter("pid.ki").as_double();
  parameter.pid.kd = ros_node.get_parameter("pid.kd").as_double();
  parameter.pid.use_anti_windup = ros_node.get_parameter("pid.use_anti_windup").as_bool();
  parameter.pid.limit = ros_node.get_parameter("pid.limit").as_double();
  parameter.frame_robot = ros_node.get_parameter("frame_robot").as_string();

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
  _controller_set_point->position.x = 1.0;
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
    rclcpp::QoS(1).best_effort(),
    std::bind(&PoseController::callbackCurrentPose, this, std::placeholders::_1)
  );
  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "twist_output", rclcpp::QoS(1).reliable()
  );
  _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  _stamp_last_processed = get_clock()->now();
}

PoseController::~PoseController()
{

}

static AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

void PoseController::callbackCurrentPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg)
{
  geometry_msgs::msg::TransformStamped t_sensor_to_base_link;
  geometry_msgs::msg::PoseStamped pose_in_base_link;

  try {
    t_sensor_to_base_link = _tf_buffer->lookupTransform(
      _parameter.frame_robot, pose_msg->header.frame_id, tf2::TimePointZero
    );
    pose_in_base_link = _tf_buffer->transform(*pose_msg, _parameter.frame_robot);
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

  const auto now = get_clock()->now();
  const double dt = (now - _stamp_last_processed).seconds();

  // Linear X
  _controller_output->linear.x = _controller[0](_controller_set_point->position.x, pose_in_base_link.pose.position.x, dt);

  // Linear Y
  _controller_output->linear.y = _controller[1](_controller_set_point->position.y, pose_in_base_link.pose.position.y, dt);  

  // Yaw Orientation
  const auto q_yaw_feedback = pose_in_base_link.pose.orientation;
  const AnglePiToPi yaw_feedback = quaternion_to_yaw(q_yaw_feedback);
  const AnglePiToPi yaw_set_point = quaternion_to_yaw(_controller_set_point->orientation);

  _controller_output->angular.z = _controller[2](yaw_set_point, yaw_feedback, dt);

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