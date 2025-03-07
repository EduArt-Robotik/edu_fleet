#include "pose_controller_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <edu_fleet/sensor_model/message_converting.hpp>

#include <Eigen/Geometry>

#include <rclcpp/logging.hpp>
#include <rclcpp/executors.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <functional>
#include <memory>

namespace eduart {
namespace fleet {

using namespace std::chrono_literals;

PoseController::Parameter PoseController::get_parameter(rclcpp::Node &ros_node)
{
  PoseController::Parameter parameter;

  // linear
  ros_node.declare_parameter<double>("pid.linear.kp", parameter.pid_linear.kp);
  ros_node.declare_parameter<double>("pid.linear.ki", parameter.pid_linear.ki);
  ros_node.declare_parameter<double>("pid.linear.kd", parameter.pid_linear.kd);
  ros_node.declare_parameter<bool>("pid.linear.use_anti_windup", parameter.pid_linear.use_anti_windup);
  ros_node.declare_parameter<double>("pid.linear.limit", parameter.pid_linear.limit);
  ros_node.declare_parameter<double>("pid.linear.input_filter_weight", parameter.pid_linear.input_filter_weight);
  // angular
  ros_node.declare_parameter<double>("pid.angular.kp", parameter.pid_angular.kp);
  ros_node.declare_parameter<double>("pid.angular.ki", parameter.pid_angular.ki);
  ros_node.declare_parameter<double>("pid.angular.kd", parameter.pid_angular.kd);
  ros_node.declare_parameter<bool>("pid.angular.use_anti_windup", parameter.pid_angular.use_anti_windup);
  ros_node.declare_parameter<double>("pid.angular.limit", parameter.pid_angular.limit);
  ros_node.declare_parameter<double>("pid.angular.input_filter_weight", parameter.pid_angular.input_filter_weight);
  // input filter
  ros_node.declare_parameter<bool>("input.filter.enable", parameter.input_filter.enable);
  ros_node.declare_parameter<double>("input.filter.weight", parameter.input_filter.weight);
  // input timeout
  ros_node.declare_parameter<bool>("input.timeout.enable", parameter.input_timeout.enable);
  ros_node.declare_parameter<int>("input.timeout.value", parameter.input_timeout.timeout_ms.count());
  // general parameter
  ros_node.declare_parameter<std::string>("target_frame_id", parameter.target_frame_id);

  // linear
  parameter.pid_linear.kp = ros_node.get_parameter("pid.linear.kp").as_double();
  parameter.pid_linear.ki = ros_node.get_parameter("pid.linear.ki").as_double();
  parameter.pid_linear.kd = ros_node.get_parameter("pid.linear.kd").as_double();
  parameter.pid_linear.use_anti_windup = ros_node.get_parameter("pid.linear.use_anti_windup").as_bool();
  parameter.pid_linear.limit = ros_node.get_parameter("pid.linear.limit").as_double();
  parameter.pid_linear.input_filter_weight = ros_node.get_parameter("pid.linear.input_filter_weight").as_double();
  // angular
  parameter.pid_angular.kp = ros_node.get_parameter("pid.angular.kp").as_double();
  parameter.pid_angular.ki = ros_node.get_parameter("pid.angular.ki").as_double();
  parameter.pid_angular.kd = ros_node.get_parameter("pid.angular.kd").as_double();
  parameter.pid_angular.use_anti_windup = ros_node.get_parameter("pid.angular.use_anti_windup").as_bool();
  parameter.pid_angular.limit = ros_node.get_parameter("pid.angular.limit").as_double();
  parameter.pid_angular.input_filter_weight = ros_node.get_parameter("pid.angular.input_filter_weight").as_double();
  // input filter
  parameter.input_filter.enable = ros_node.get_parameter("input.filter.enable").as_bool();
  parameter.input_filter.weight = ros_node.get_parameter("input.filter.weight").as_double();
  // input timeout
  parameter.input_timeout.enable = ros_node.get_parameter("input.timeout.enable").as_bool();
  parameter.input_timeout.timeout_ms = std::chrono::milliseconds(
    ros_node.get_parameter("input.timeout.value").as_int());
  // general parameter
  parameter.target_frame_id = ros_node.get_parameter("target_frame_id").as_string();

  return parameter;
}

PoseController::PoseController()
  : rclcpp::Node("pose_controller")
  , _parameter(get_parameter(*this))
  , _set_point(std::make_unique<geometry_msgs::msg::Pose>())
  , _feedback(std::make_unique<geometry_msgs::msg::Pose>())
  , _output(std::make_unique<geometry_msgs::msg::Twist>())
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  , _tf_buffer(std::make_unique<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_unique<tf2_ros::TransformListener>(*_tf_buffer))
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


  // input filter
  _low_pass_orientation = std::make_unique<QuaternionLowPassFilter>(
    QuaternionLowPassFilter::Parameter{_parameter.input_filter.weight}
  );
  _low_pass_position = std::make_unique<PositionLowPassFilter>(
    PositionLowPassFilter::Parameter{_parameter.input_filter.weight}
  );


  // creating subscriptions and publisher
  _sub_current_odometry = create_subscription<nav_msgs::msg::Odometry>(
    "odometry_feedback",
    rclcpp::QoS(2).best_effort(),
    std::bind(&PoseController::callbackCurrentOdometry, this, std::placeholders::_1)
  );
  _sub_current_pose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
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


  // initialize
  _stamp_last_processed = get_clock()->now();


  // input timeout
  if (_parameter.input_timeout.enable) {
    _timer_checking_timeout = create_timer(
      50ms, std::bind(&PoseController::checkIfTimeoutOccurred, this)
    );
  }
}

PoseController::~PoseController()
{

}

void PoseController::callbackCurrentOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> odometry_msg)
{
  // only work in given frame id to avoid tf transform that takes too long...
  if (odometry_msg->header.frame_id != _parameter.target_frame_id) {
    RCLCPP_ERROR(get_logger(), "received odometry must be in frame \"%s\"", _parameter.target_frame_id.c_str());
    return;
  }

  processCurrentPose(odometry_msg->pose.pose);
}

void PoseController::callbackCurrentPose(std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> pose_msg)
{
  try {
    const auto pose_transformed = _tf_buffer->transform(*pose_msg, _parameter.target_frame_id);
    processCurrentPose(pose_transformed.pose.pose);
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception was thrown during processing current pose. what = %s", ex.what());
  }
}

void PoseController::processCurrentPose(const geometry_msgs::msg::Pose& pose_msg)
{
  // filter input if enabled
  if (_parameter.input_filter.enable) {
    // update filter
    (*_low_pass_position)(pose_msg.position);
    (*_low_pass_orientation)(pose_msg.orientation);

    // use filter output as feedback
    _feedback->position = _low_pass_position->getValue();
    _feedback->orientation = _low_pass_orientation->getValue();
  }
  // default: use input as feedback
  else {
    *_feedback = pose_msg;
  }

  _stamp_last_feedback_received = get_clock()->now();
  process();
}

void PoseController::callbackTargetPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg)
{
  // only work in given frame id to avoid tf transform that takes too long...
  if (pose_msg->header.frame_id != _parameter.target_frame_id) {
    RCLCPP_ERROR(get_logger(), "received target pose must be in frame \"%s\"", _parameter.target_frame_id.c_str());
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
  if (std::abs(_output->linear.x)  < 0.01) _output->linear.x  = 0.0;
  if (std::abs(_output->linear.y)  < 0.01) _output->linear.y  = 0.0;
  if (std::abs(_output->angular.z) < 0.01) _output->angular.z = 0.0;

  // Publishing Result
  _pub_twist->publish(*_output);
  _stamp_last_processed = now;
}

void PoseController::checkIfTimeoutOccurred()
{
  const auto now = get_clock()->now();
  const auto timeout = rclcpp::Duration::from_nanoseconds(
    _parameter.input_timeout.timeout_ms.count() /* convert ms into ns */ * 1000);

  if (now - _stamp_last_feedback_received > timeout) {
    std::cout << "TIMEOUT!!!" << std::endl;
    // timeout occurred --> reset output and publish it
    // note: it will publish with the frequency this function is called until new input pose was processed.
    _output->linear.x = 0.0;
    _output->linear.y = 0.0;
    _output->angular.z = 0.0;

    _pub_twist->publish(*_output);
  }
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