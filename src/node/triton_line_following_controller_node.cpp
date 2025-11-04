#include "triton_line_following_controller_node.hpp"

#include <rclcpp/executors.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <edu_robot/algorithm/rotation.hpp>

namespace eduart {
namespace fleet {

static geometry_msgs::msg::PoseArray transform_poses(
  const geometry_msgs::msg::PoseArray& poses_in, const std::string target_frame_id, const std::string& sensor_frame_id,
  const tf2_ros::Buffer& tf_buffer, const rclcpp::Logger& logger)
{
  geometry_msgs::msg::PoseArray poses_out;
  poses_out.header.stamp = poses_in.header.stamp;
  poses_out.header.frame_id = target_frame_id;

  try {
    const auto transform = tf_buffer.lookupTransform(
      target_frame_id, sensor_frame_id, poses_in.header.stamp,
      rclcpp::Duration::from_seconds(0.1)
    );
    geometry_msgs::msg::Pose pose_out;

    for (const auto& pose_in : poses_in.poses) {
      tf2::doTransform(pose_in, pose_out, transform);

      std::cout << "Transformed Pose: x = " << pose_out.position.x
                << ", y = " << pose_out.position.y
                << ", z = " << pose_out.position.z << std::endl;
      poses_out.poses.push_back(pose_out);
    }
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger, "Could not transform poses to robot base frame \"%s\": %s", target_frame_id.c_str(), ex.what());
  }

  return poses_out;
}

static Eigen::Vector3d calculate_error_vector(
  const geometry_msgs::msg::Pose& pose_feedback, const geometry_msgs::msg::Pose& pose_set_point)
{
  // given poses are in map frame --> transform into robot frame
  // rotate coordinates in robot frame and subtract feedback from it
  const Eigen::Vector2d position_feedback(pose_feedback.position.x, pose_feedback.position.y);
  const Eigen::Vector2d position_set_point(pose_set_point.position.x, pose_set_point.position.y);
  const robot::AnglePiToPi yaw_feedback  = robot::algorithm::quaternion_to_yaw(pose_feedback.orientation);
  const robot::AnglePiToPi yaw_set_point = robot::algorithm::quaternion_to_yaw(pose_set_point.orientation);
  const Eigen::Rotation2Dd R(-yaw_feedback);

  // e = R * (set_point - feedback)
  const Eigen::Vector2d position_error = R * (position_set_point - position_feedback);
  const robot::AnglePiToPi yaw_error = yaw_set_point - yaw_feedback;

  return Eigen::Vector3d(position_error.x(), position_error.y(), yaw_error.radian());
}

TritonLineFollowingController::Parameter TritonLineFollowingController::get_parameter(
  const Parameter& default_parameter, rclcpp_lifecycle::LifecycleNode& ros_node)
{
  ros_node.declare_parameter<double>("pid.y.kp", default_parameter.pid.y.kp);
  ros_node.declare_parameter<double>("pid.y.limit", default_parameter.pid.y.limit);
  ros_node.declare_parameter<double>("pid.heading.kp", default_parameter.pid.heading.kp);
  ros_node.declare_parameter<double>("pid.heading.limit", default_parameter.pid.heading.limit);

  ros_node.declare_parameter<std::string>("target_frame_id", default_parameter.target_frame_id);
  ros_node.declare_parameter<std::string>("sensor_frame_id", default_parameter.sensor_frame_id);

  Parameter parameter = default_parameter;

  parameter.pid.y.kp = ros_node.get_parameter("pid.y.kp").as_double();
  parameter.pid.y.limit = ros_node.get_parameter("pid.y.limit").as_double();
  parameter.pid.heading.kp = ros_node.get_parameter("pid.heading.kp").as_double();
  parameter.pid.heading.limit = ros_node.get_parameter("pid.heading.limit").as_double();

  parameter.target_frame_id = ros_node.get_parameter("target_frame_id").as_string();
  parameter.sensor_frame_id = ros_node.get_parameter("sensor_frame_id").as_string();

  return parameter;
}

TritonLineFollowingController::TritonLineFollowingController()
  : rclcpp_lifecycle::LifecycleNode("triton_line_following_controller")
  , _parameter(get_parameter(Parameter(), *this))
  , _pid_y(std::make_shared<controller::Pid>(_parameter.pid.y))
  , _pid_heading(std::make_shared<controller::Pid>(_parameter.pid.heading))
  , _tf_buffer(std::make_shared<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_shared<tf2_ros::TransformListener>(*_tf_buffer))
{

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_configure(
  const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_logger(), "configuring node.");
  (void)previous_state;

  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "out/cmd_vel", rclcpp::QoS(2).reliable()
  );
  _sub_line_following = create_subscription<geometry_msgs::msg::PoseArray>(
    "in/line_following",
    rclcpp::QoS(2).best_effort(),
    std::bind(&TritonLineFollowingController::callbackLineFollowingPoses, this, std::placeholders::_1)
  );
  _data.stamp_last_processing = get_clock()->now();

  RCLCPP_INFO(get_logger(), "configured node.");

  return rclcpp_lifecycle::LifecycleNode::on_configure(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_logger(), "activating node.");

  _pid_y->reset();
  _pid_heading->reset();

  _data.stamp_last_processing = get_clock()->now();
  _data.docking_in = true;

  return rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_deactivate(
  const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_logger(), "deactivating node.");

  _data.docking_in = false;

  // publish null velocity to stop pose controlling impact.
  _pub_twist->publish(geometry_msgs::msg::Twist());

  return rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_cleanup(
  const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_logger(), "cleaning up node.");

  return rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TritonLineFollowingController::on_shutdown(
  const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_logger(), "shutting down node.");

  return rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
}

void TritonLineFollowingController::callbackLineFollowingPoses(std::shared_ptr<const geometry_msgs::msg::PoseArray> msg)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  // transform poses into robot base frame
  const auto poses_transformed = transform_poses(
    *msg, _parameter.target_frame_id, _parameter.sensor_frame_id, 
    *_tf_buffer, get_logger()
  );

  if (poses_transformed.poses.size() != 2) {
    RCLCPP_ERROR(get_logger(), "Received poses do not contain correct number of poses for line following control.");
    return;
  }

  // start processing
  const auto stamp_now = get_clock()->now();
  const double dt = std::min(0.1, (stamp_now - _data.stamp_last_processing).seconds());
  const robot::AnglePiToPi yaw_robot = robot::algorithm::quaternion_to_yaw(poses_transformed.poses[0].orientation);
  const robot::AnglePiToPi yaw_target = robot::algorithm::quaternion_to_yaw(poses_transformed.poses[1].orientation);

  RCLCPP_INFO(
    get_logger(), "robot pose x = %f, y = %f, yaw = %f.",
    poses_transformed.poses[0].position.x,
    poses_transformed.poses[0].position.y,
    yaw_robot.radian()
  );
  RCLCPP_INFO(
    get_logger(), "target pose x = %f, y = %f, yaw = %f.",
    poses_transformed.poses[1].position.x,
    poses_transformed.poses[1].position.y,
    yaw_target.radian()
  );

  const auto error_vector = calculate_error_vector(
    poses_transformed.poses[0], poses_transformed.poses[1]);
  const double error_x       = error_vector.x();
  const double error_y       = error_vector.y();
  const double error_heading = error_vector.z();

  const double vel_x    = (error_x > -0.01 ? 0.1 : 0.0);  // move forward only if target is in front of robot.
  const double vel_y    = _pid_y->process(0.0, -error_y, dt);
  const double yaw_rate = _pid_heading->process(0.0, -error_heading, dt);

  RCLCPP_INFO(get_logger(), "error x direction = %f.", error_x);
  RCLCPP_INFO(get_logger(), "error in y direction = %f.", error_y);
  RCLCPP_INFO(get_logger(), "velocity x = %f.", vel_x);
  RCLCPP_INFO(get_logger(), "velocity y = %f.", vel_y);
  RCLCPP_INFO(get_logger(), "yaw rate = %f.", yaw_rate);

  // if (error_x)

  // Finish processing.
  geometry_msgs::msg::Twist twist_out;

  twist_out.linear.x = vel_x;
  twist_out.linear.y = vel_y;
  twist_out.angular.z = yaw_rate;

  _pub_twist->publish(twist_out);

  _data.stamp_last_processing = stamp_now;
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
