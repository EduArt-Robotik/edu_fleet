#include "pose_controller_node.hpp"

#include <edu_fleet/sensor_model/message_converting.hpp>

#include <Eigen/Geometry>

#include <rclcpp/logging.hpp>
#include <rclcpp/executors.hpp>

#include <functional>
#include <memory>

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

  return parameter;
}

PoseController::PoseController()
  : rclcpp::Node("pose_controller")
  , _parameter(get_parameter(*this))
  , _set_point(std::make_unique<geometry_msgs::msg::Pose>())
  , _feedback(std::make_unique<geometry_msgs::msg::Pose>())
  , _output(std::make_unique<geometry_msgs::msg::Twist>())
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  // instantiate and initialize controller
  // position x
  auto controller = std::make_unique<controller::Pid>();
  controller->parameter = _parameter.pid_linear;
  controller->reset();
  _controller[0] = std::move(controller);

  // position y
  controller = std::make_unique<controller::Pid>();
  controller->parameter = _parameter.pid_linear;
  controller->reset();  
  _controller[1] = std::move(controller);

  // orientation yaw
  controller = std::make_unique<controller::Pid>();
  controller->parameter = _parameter.pid_angular;
  controller->reset();
  _controller[2] = std::move(controller);  

  // creating subscriptions and publisher
  _sub_current_pose = create_subscription<nav_msgs::msg::Odometry>(
    "pose_feedback",
    rclcpp::QoS(2).best_effort(),
    std::bind(&PoseController::callbackCurrentPose, this, std::placeholders::_1)
  );
  _sub_target_pose = create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose_target", 
    rclcpp::QoS(2).best_effort(),
    std::bind(&PoseController::callbackTargetPose, this, std::placeholders::_1)
  );
  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "twist_output", rclcpp::QoS(1).reliable()
  );


  // initialize variables
  _stamp_last_processed = get_clock()->now();
}

PoseController::~PoseController()
{

}

void PoseController::callbackCurrentPose(std::shared_ptr<const nav_msgs::msg::Odometry> odometry_msg)
{
  // only work in given frame id to avoid tf transform that takes too long...
  if (odometry_msg->header.frame_id != _parameter.frame_id) {
    RCLCPP_ERROR(get_logger(), "received pose must be in frame \"%s\"", _parameter.frame_id.c_str());
    return;
  }

  *_feedback = odometry_msg->pose.pose;
  process();
}

void PoseController::callbackTargetPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg)
{
  // only work in given frame id to avoid tf transform that takes too long...
  if (pose_msg->header.frame_id != _parameter.frame_id) {
    RCLCPP_ERROR(get_logger(), "received pose must be in frame \"%s\"", _parameter.frame_id.c_str());
    return;
  }

  *_set_point = pose_msg->pose;
  process();
}

void PoseController::process()
{
  using sensor_model::message_converting;

  // start processing
  // RCLCPP_INFO(get_logger(), "current set point: x = %f, y = %f, yaw = %f", _parameter.set_point.x, _parameter.set_point.y, _parameter.set_point.yaw);  
  // RCLCPP_INFO(get_logger(), "received pose: x = %f, y = %f, z = %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

  // For Debugging
  // geometry_msgs::msg::TransformStamped qr_code_transform;
  // qr_code_transform.header.frame_id = getRobotName() + '/' + _parameter.frame_robot;
  // qr_code_transform.header.stamp = get_clock()->now();
  // qr_code_transform.child_frame_id =  getRobotName() + '/' + "pose";
  // qr_code_transform.transform.translation.x = pose_in_base_link.pose.position.x;
  // qr_code_transform.transform.translation.y = pose_in_base_link.pose.position.y;
  // qr_code_transform.transform.translation.z = pose_in_base_link.pose.position.z;
  // qr_code_transform.transform.rotation = pose_in_base_link.pose.orientation;
  // _tf_broadcaster->sendTransform(qr_code_transform);

  // Process pose controller on each dimension.
  // \todo setup NTP server for robot fleet and use msg time.
  const auto now = get_clock()->now();
  // Keep dt smaller than 100ms. Bigger values are bad for the PID controller.
  const double dt = std::max(std::min((now - _stamp_last_processed).seconds(), 0.1), 1e-3);
  // dt == 0 leads to nan in pid controller...

  // Yaw Orientation
  const robot::AnglePiToPi yaw_feedback  = sensor_model::message_converting<>::quaternion_to_yaw(_feedback->orientation);
  const robot::AnglePiToPi yaw_set_point = sensor_model::message_converting<>::quaternion_to_yaw(_set_point->orientation);
  const robot::AnglePiToPi yaw_target = yaw_feedback - yaw_set_point;
  RCLCPP_INFO(get_logger(), "feedback yaw: %f, set point yaw: %f", yaw_feedback.radian(), yaw_set_point.radian());
  RCLCPP_INFO(get_logger(), "target yaw: %f", yaw_target.radian());
  _output->angular.z = _controller[2]->process(0.0, yaw_target, dt);

  // Linear
  const Eigen::Vector2f position_feedback(_feedback->position.x, _feedback->position.y);
  RCLCPP_INFO(get_logger(), "feedback point: x = %f, y = %f", position_feedback.x(), position_feedback.y());
  const Eigen::Vector2f position_set_point(_set_point->position.x, _set_point->position.y);
  RCLCPP_INFO(get_logger(), "set point: x = %f, y = %f", position_set_point.x(), position_set_point.y());
  const Eigen::Vector2f target_point = position_feedback - position_set_point;
  RCLCPP_INFO(get_logger(), "target point: x = %f, y = %f", target_point.x(), target_point.y());

  Eigen::Vector2f output(
    // Linear X
    _controller[0]->process(0.0f, target_point.x(), dt),
    // Linear Y
    _controller[1]->process(0.0f, target_point.y(), dt)
  );
  // rotate output in robot frame
  output = Eigen::Rotation2Df(-yaw_feedback) * output;

  _output->linear.x = output.x();
  _output->linear.y = output.y();

  // \todo replace hack with proper implementation
  if (std::abs(_output->linear.x)  < 0.01) _output->linear.x = 0.0;
  if (std::abs(_output->linear.y)  < 0.01) _output->linear.y = 0.0;
  if (std::abs(_output->angular.z) < 0.01) _output->angular.z = 0.0;

  // Publishing Result
  _pub_twist->publish(*_output);
  _stamp_last_processed = now;
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