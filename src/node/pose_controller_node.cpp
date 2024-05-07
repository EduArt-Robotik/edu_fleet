#include "pose_controller_node.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
  ros_node.declare_parameter<std::string>("reference_robot_frame", parameter.reference_robot_frame);
  ros_node.declare_parameter<std::string>("marker_frame", parameter.marker_frame);

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
  parameter.reference_robot_frame = ros_node.get_parameter("reference_robot_frame").as_string();
  parameter.marker_frame = ros_node.get_parameter("marker_frame").as_string();

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
  _sub_aruco_marker_detection = create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    "marker_detection",
    rclcpp::QoS(1).best_effort(),
    std::bind(&PoseController::callbackMarkerDetection, this, std::placeholders::_1)
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

static robot::AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q)
{
  return atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

void PoseController::callbackCurrentPose(std::shared_ptr<const geometry_msgs::msg::PoseStamped> pose_msg)
{
  process(*pose_msg);
}

void PoseController::callbackMarkerDetection(std::shared_ptr<const aruco_opencv_msgs::msg::ArucoDetection> marker_msg)
{
  for (const auto& marker : marker_msg->markers) {
    if (marker.marker_id == 0) {
      geometry_msgs::msg::PoseStamped pose;

      pose.pose = marker.pose;
      pose.header = marker_msg->header;

      try {
        // const auto transform = _tf_buffer->lookupTransform(
        //   _parameter.marker_frame, _parameter.reference_robot_frame, marker_msg->header.stamp
        // );
        // const auto transform = _tf_buffer->lookupTransform(
        //   _parameter.reference_robot_frame, _parameter.marker_frame, marker_msg->header.stamp
        // );
        // @todo: transformation seems a bit messed up. At the moment it is a hack for fleet application. Fix me!
        Eigen::Quaterniond rotation;
        rotation = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond orientation(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        orientation = orientation * rotation;
 
        pose.pose.orientation.w = orientation.w();
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();

        Eigen::Vector3d translation = orientation * Eigen::Vector3d(0.17, 0.0, 0.0);
        pose.pose.position.x += translation.x();
        pose.pose.position.y += translation.y();
        pose.pose.position.z += translation.z();                
      }
      catch(const tf2::TransformException & ex) {
        // no transform available --> do nothing
        RCLCPP_ERROR(
          get_logger(), "could not transform %s to %s: %s", _parameter.reference_robot_frame.c_str(), 
          _parameter.marker_frame.c_str(), ex.what()
        );
        break;
      }

      process(pose);
      break;
    }
  }
}

void PoseController::process(const geometry_msgs::msg::PoseStamped& pose)
{
  RCLCPP_INFO(get_logger(), "current set point: x = %f, y = %f, yaw = %f", _parameter.set_point.x, _parameter.set_point.y, _parameter.set_point.yaw);  
  RCLCPP_INFO(get_logger(), "received pose: x = %f, y = %f, z = %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  // Bring pose in robot coordinate system.
  geometry_msgs::msg::PoseStamped pose_in_base_link;

  try {
    pose_in_base_link = _tf_buffer->transform(pose, getRobotName() + '/' + _parameter.frame_robot);
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      get_logger(), "could not transform %s to %s: %s", (getRobotName() + '/' + _parameter.frame_robot).c_str(), 
      pose.header.frame_id.c_str(), ex.what()
    );
    return;
  }

  RCLCPP_INFO(get_logger(), "in base link: x = %f, y = %f, z = %f", pose_in_base_link.pose.position.x, pose_in_base_link.pose.position.y, pose_in_base_link.pose.position.z);

  // For Debugging
  geometry_msgs::msg::TransformStamped qr_code_transform;
  qr_code_transform.header.frame_id = getRobotName() + '/' + _parameter.frame_robot;
  qr_code_transform.header.stamp = get_clock()->now();
  qr_code_transform.child_frame_id =  getRobotName() + '/' + "pose";
  qr_code_transform.transform.translation.x = pose_in_base_link.pose.position.x;
  qr_code_transform.transform.translation.y = pose_in_base_link.pose.position.y;
  qr_code_transform.transform.translation.z = pose_in_base_link.pose.position.z;
  qr_code_transform.transform.rotation = pose_in_base_link.pose.orientation;
  _tf_broadcaster->sendTransform(qr_code_transform);

  // Process pose controller on each dimension.
  const Eigen::Vector2f position_in_base_link(pose_in_base_link.pose.position.x, pose_in_base_link.pose.position.y);
  // \todo setup NTP server for robot fleet and use msg time.
  const auto now = get_clock()->now();
  // Keep dt smaller than 100ms. Bigger values are bad for the PID controller.
  const double dt = std::min((now - _stamp_last_processed).seconds(), 0.1);

  // Yaw Orientation
  const auto q_yaw_feedback = pose_in_base_link.pose.orientation;
  const robot::AnglePiToPi yaw_feedback = quaternion_to_yaw(q_yaw_feedback);
  RCLCPP_INFO(get_logger(), "yaw feedback = %f, yaw set point = %f", yaw_feedback.radian(), _parameter.set_point.yaw);
  const robot::AnglePiToPi yaw_set_point = _parameter.set_point.yaw;

  _controller_output->angular.z = _controller[2](-yaw_set_point, -yaw_feedback, dt);

  // Linear
  const Eigen::Vector2f set_point = Eigen::Rotation2Df(yaw_feedback - _parameter.set_point.yaw) * Eigen::Vector2f(_parameter.set_point.x, _parameter.set_point.y);
  RCLCPP_INFO(get_logger(), "set point: x = %f, y = %f", set_point.x(), set_point.y());
  const Eigen::Vector2f target_point = position_in_base_link - set_point;
  RCLCPP_INFO(get_logger(), "target point: x = %f, y = %f", target_point.x(), target_point.y());
  // Linear X
  _controller_output->linear.x = _controller[0](0.0f, -target_point.x(), dt);

  // Linear Y
  _controller_output->linear.y = _controller[1](0.0f, -target_point.y(), dt); 

  // \todo replace hack with proper implementation
  if (std::abs(_controller_output->linear.x) < 0.01) _controller_output->linear.x = 0.0;
  if (std::abs(_controller_output->linear.y) < 0.01) _controller_output->linear.y = 0.0;
  if (std::abs(_controller_output->angular.z) < 0.01) _controller_output->angular.z = 0.0;

  // Publishing Result
  _pub_twist->publish(*_controller_output);
  _stamp_last_processed = now;
}

void PoseController::getTransform()
{
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  using ResponseFuture = rclcpp::Client<edu_swarm::srv::GetTransform>::SharedFutureWithRequest;

  auto request = std::make_shared<edu_swarm::srv::GetTransform::Request>();
  // request->from_robot = _parameter.robot_name;
  request->from_robot = getRobotName();
  request->to_robot = _parameter.reference_robot_name;

  _srv_client_get_transform->async_send_request(request, [this](ResponseFuture future){
    const auto response = future.get();
    const auto t = response.second->t;

    if (t.cols != 3 || t.rows != 3) {
      RCLCPP_ERROR(get_logger(), "received transform is null");
      return;
    }

    _parameter.set_point.x = t.data[0 * t.cols + 2];
    _parameter.set_point.y = t.data[1 * t.cols + 2];
    _parameter.set_point.yaw = std::asin(t.data[1 * t.cols + 0]);

    RCLCPP_INFO(get_logger(), "received new set point: x = %f, y = %f, yaw = %f", _parameter.set_point.x, _parameter.set_point.y, _parameter.set_point.yaw);
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