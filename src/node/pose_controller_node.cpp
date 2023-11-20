#include "pose_controller_node.hpp"
#include "angle.hpp"
#include "edu_swarm/srv/detail/get_transform__struct.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/executors.hpp>

#include <functional>

namespace eduart {
namespace fleet {

using namespace std::chrono_literals;

PoseController::Parameter PoseController::get_parameter(rclcpp::Node &ros_node)
{
  PoseController::Parameter parameter;

  ros_node.declare_parameter<double>("pid.linear.kp", parameter.pid_linear.kp);
  ros_node.declare_parameter<double>("pid.linear.ki", parameter.pid_linear.ki);
  ros_node.declare_parameter<double>("pid.linear.kd", parameter.pid_linear.kd);
  ros_node.declare_parameter<bool>("pid.linear.use_anti_windup", parameter.pid_linear.use_anti_windup);
  ros_node.declare_parameter<double>("pid.linear.limit", parameter.pid_linear.limit);
  ros_node.declare_parameter<double>("pid.linear.input_filter_weight", parameter.pid_linear.input_filter_weight);
  ros_node.declare_parameter<double>("pid.angular.kp", parameter.pid_angular.kp);
  ros_node.declare_parameter<double>("pid.angular.ki", parameter.pid_angular.ki);
  ros_node.declare_parameter<double>("pid.angular.kd", parameter.pid_angular.kd);
  ros_node.declare_parameter<bool>("pid.angular.use_anti_windup", parameter.pid_angular.use_anti_windup);
  ros_node.declare_parameter<double>("pid.angular.limit", parameter.pid_angular.limit);
  ros_node.declare_parameter<double>("pid.angular.input_filter_weight", parameter.pid_angular.input_filter_weight);

  ros_node.declare_parameter<std::string>("frame_robot", parameter.frame_robot);
  ros_node.declare_parameter<std::string>("reference_robot_name", parameter.reference_robot_name);

  ros_node.declare_parameter<double>("set_point.x", parameter.set_point.x);
  ros_node.declare_parameter<double>("set_point.y", parameter.set_point.y);
  ros_node.declare_parameter<double>("set_point.yaw", parameter.set_point.yaw);

  parameter.pid_linear.kp = ros_node.get_parameter("pid.linear.kp").as_double();
  parameter.pid_linear.ki = ros_node.get_parameter("pid.linear.ki").as_double();
  parameter.pid_linear.kd = ros_node.get_parameter("pid.linear.kd").as_double();
  parameter.pid_linear.use_anti_windup = ros_node.get_parameter("pid.linear.use_anti_windup").as_bool();
  parameter.pid_linear.limit = ros_node.get_parameter("pid.linear.limit").as_double();
  parameter.pid_linear.input_filter_weight = ros_node.get_parameter("pid.linear.input_filter_weight").as_double();
  parameter.pid_angular.kp = ros_node.get_parameter("pid.angular.kp").as_double();
  parameter.pid_angular.ki = ros_node.get_parameter("pid.angular.ki").as_double();
  parameter.pid_angular.kd = ros_node.get_parameter("pid.angular.kd").as_double();
  parameter.pid_angular.use_anti_windup = ros_node.get_parameter("pid.angular.use_anti_windup").as_bool();
  parameter.pid_angular.limit = ros_node.get_parameter("pid.angular.limit").as_double();
  parameter.pid_angular.input_filter_weight = ros_node.get_parameter("pid.angular.input_filter_weight").as_double();

  parameter.frame_robot = ros_node.get_parameter("frame_robot").as_string();
  parameter.reference_robot_name = ros_node.get_parameter("reference_robot_name").as_string();

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
  _srv_client_get_transform = create_client<edu_swarm::srv::GetTransform>("/get_transform");
  _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  _stamp_last_processed = get_clock()->now();
  for (auto& controller : _controller) {
    controller.reset();
  }
  _timer_get_transform = create_wall_timer(2s, std::bind(&PoseController::getTransform, this));
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
  // Bring pose in robot coordinate system.
  geometry_msgs::msg::PoseStamped pose_in_base_link;

  try {
    pose_in_base_link = _tf_buffer->transform(*pose_msg, getRobotName() + '/' + _parameter.frame_robot);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      get_logger(), "Could not transform %s to %s: %s", (getRobotName() + '/' + _parameter.frame_robot).c_str(), 
      pose_msg->header.frame_id.c_str(), ex.what()
    );
    return;
  }

  // For Debugging
  // geometry_msgs::msg::TransformStamped qr_code_transform;
  // qr_code_transform.header.frame_id = _parameter.frame_robot;
  // qr_code_transform.header.stamp = get_clock()->now();
  // qr_code_transform.child_frame_id = "eduard_red";
  // qr_code_transform.transform.translation.x = pose_in_base_link.pose.position.x;
  // qr_code_transform.transform.translation.y = pose_in_base_link.pose.position.y;
  // qr_code_transform.transform.translation.z = pose_in_base_link.pose.position.z;
  // qr_code_transform.transform.rotation = pose_in_base_link.pose.orientation;
  // _tf_broadcaster->sendTransform(qr_code_transform);

  // Process pose controller on each dimension.
  const Eigen::Vector2f position_in_base_link(pose_in_base_link.pose.position.x, pose_in_base_link.pose.position.y);
  const auto now = get_clock()->now();
  // Keep dt smaller than 100ms. Bigger values are bad for the PID controller.
  const double dt = std::min((now - _stamp_last_processed).seconds(), 0.1);

  // Yaw Orientation
  const auto q_yaw_feedback = pose_in_base_link.pose.orientation;
  const AnglePiToPi yaw_feedback = quaternion_to_yaw(q_yaw_feedback);
  const AnglePiToPi yaw_set_point = _parameter.set_point.yaw;

  _controller_output->angular.z = _controller[2](-yaw_set_point, -yaw_feedback, dt);

  // Linear
  const Eigen::Vector2f set_point = Eigen::Rotation2Df(yaw_feedback - _parameter.set_point.yaw) * Eigen::Vector2f(_parameter.set_point.x, _parameter.set_point.y);
  const Eigen::Vector2f target_point = position_in_base_link - set_point;

  // Linear X
  _controller_output->linear.x = _controller[0](0.0f, -target_point.x(), dt);

  // Linear Y
  _controller_output->linear.y = _controller[1](0.0f, -target_point.y(), dt); 

  // \todo replace hack with proper implementation
  if (std::abs(_controller_output->linear.x) < 0.02) _controller_output->linear.x = 0.0;
  if (std::abs(_controller_output->linear.y) < 0.02) _controller_output->linear.y = 0.0;
  if (std::abs(_controller_output->angular.z) < 0.02) _controller_output->angular.z = 0.0;

  // Publishing Result
  _pub_twist->publish(*_controller_output);
  _stamp_last_processed = now;
}

void PoseController::getTransform()
{
  using ResponseFuture = rclcpp::Client<edu_swarm::srv::GetTransform>::SharedFutureWithRequest;

  auto request = std::make_shared<edu_swarm::srv::GetTransform::Request>();
  // request->from_robot = _parameter.robot_name;
  request->from_robot = getRobotName();
  request->to_robot = _parameter.reference_robot_name;

  _srv_client_get_transform->async_send_request(request, [this](ResponseFuture future){
    const auto response = future.get();
    const auto t = response.second->t;

    if (t.cols != 3 || t.rows != 3) {
      RCLCPP_ERROR(get_logger(), "Received transform is null");
      return;
    }

    _parameter.set_point.x = t.data[0 * t.cols + 2];
    _parameter.set_point.y = t.data[1 * t.cols + 2];
    _parameter.set_point.yaw = std::asin(t.data[1 * t.cols + 0]);
  });
}

std::string PoseController::getRobotName() const
{
  // remove slash at the beginning
  std::string frame_id_prefix(get_effective_namespace().begin() + 1, get_effective_namespace().end());
  return frame_id_prefix;
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