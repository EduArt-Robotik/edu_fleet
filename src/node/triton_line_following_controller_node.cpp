#include "triton_line_following_controller_node.hpp"

#include <rclcpp/executors.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <edu_robot/algorithm/rotation.hpp>

namespace eduart {
namespace fleet {

/**
 * \brief Transfrom poses into target frame, most likely the robot frame. 
*/
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
      poses_out.poses.push_back(pose_out);
    }
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger, "Could not transform poses to robot base frame \"%s\": %s", target_frame_id.c_str(), ex.what());
  }

  return poses_out;
}

/**
 * \brief Calculates error vector with pose_feedback as reference frame. Error vector is rotated into pose_feedback frame.
 */
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

static double determine_velocity_x(const bool docking_in, const bool docking_out, const double v_x)
{
  if (docking_in == true && docking_out == false) {
    return v_x;
  }
  else if (docking_in == false && docking_out == true) {
    return -v_x;
  }
  else {
    return 0.0;
  }
}

TritonLineFollowingController::Parameter TritonLineFollowingController::get_parameter(
  const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  ros_node.declare_parameter<double>("pid.y.kp", default_parameter.pid.y.kp);
  ros_node.declare_parameter<double>("pid.y.limit", default_parameter.pid.y.limit);
  ros_node.declare_parameter<double>("pid.heading.kp", default_parameter.pid.heading.kp);
  ros_node.declare_parameter<double>("pid.heading.limit", default_parameter.pid.heading.limit);

  ros_node.declare_parameter<double>("docking_end_error", default_parameter.docking_end_error);
  ros_node.declare_parameter<int>("stop_time", static_cast<int>(default_parameter.stop_time.count()));

  ros_node.declare_parameter<std::string>("target_frame_id", default_parameter.target_frame_id);
  ros_node.declare_parameter<std::string>("sensor_frame_id", default_parameter.sensor_frame_id);

  Parameter parameter = default_parameter;

  parameter.pid.y.kp = ros_node.get_parameter("pid.y.kp").as_double();
  parameter.pid.y.limit = ros_node.get_parameter("pid.y.limit").as_double();
  parameter.pid.heading.kp = ros_node.get_parameter("pid.heading.kp").as_double();
  parameter.pid.heading.limit = ros_node.get_parameter("pid.heading.limit").as_double();

  parameter.docking_end_error = ros_node.get_parameter("docking_end_error").as_double();
  parameter.stop_time = std::chrono::milliseconds(ros_node.get_parameter("stop_time").as_int());

  parameter.target_frame_id = ros_node.get_parameter("target_frame_id").as_string();
  parameter.sensor_frame_id = ros_node.get_parameter("sensor_frame_id").as_string();

  return parameter;
}

TritonLineFollowingController::TritonLineFollowingController()
  : rclcpp::Node("triton_line_following_controller")
  , _parameter(get_parameter(Parameter(), *this))
  , _pid_y(std::make_shared<controller::Pid>(_parameter.pid.y))
  , _pid_heading(std::make_shared<controller::Pid>(_parameter.pid.heading))
  , _tf_buffer(std::make_shared<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_shared<tf2_ros::TransformListener>(*_tf_buffer))
{
  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "out/cmd_vel", rclcpp::QoS(2).reliable()
  );
  _sub_line_following = create_subscription<geometry_msgs::msg::PoseArray>(
    "in/line_following",
    rclcpp::QoS(2).best_effort(),
    std::bind(&TritonLineFollowingController::callbackLineFollowingPoses, this, std::placeholders::_1)
  );
  _client_set_line_following = create_client<accerion_driver_msgs::srv::SetClusterMode>(
    "set_line_following_mode"
  );
  _action_server = rclcpp_action::create_server<edu_fleet::action::TritonDocking>(
    this,
    "docking",
    std::bind(&TritonLineFollowingController::callbackAcceptDocking, this, std::placeholders::_1, std::placeholders::_2),
    [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::TritonDocking>> goal_handle) {
      // accept all cancel requests
      (void)goal_handle;

      RCLCPP_INFO(get_logger(), "received goal cancel request");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&TritonLineFollowingController::callbackDocking, this, std::placeholders::_1)
  );
  _data.stamp_last_processing = get_clock()->now();
}

rclcpp_action::GoalResponse TritonLineFollowingController::callbackAcceptDocking(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const edu_fleet::action::TritonDocking::Goal> goal)
{
  (void)uuid;

  if (_data.docking_in == true || _data.docking_out == true || _data.at_endposition == true) {
    RCLCPP_WARN(get_logger(), "cannot accept new docking goal, docking is already in progress.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "activating node.");
  _pid_y->reset();
  _pid_heading->reset();

  _data.stamp_last_processing = get_clock()->now();
  _data.docking_in = true;
  _data.docking_out = false;
  _data.at_endposition = false;

  enableLineFollowingMode(goal->cluster_id);
  _data.active_cluster_id = goal->cluster_id;
  RCLCPP_INFO(get_logger(), "started docking in.");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void TritonLineFollowingController::callbackDocking(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::TritonDocking>> goal_handle)
{
  _data.goal_handle = goal_handle;
}

void TritonLineFollowingController::callbackLineFollowingPoses(std::shared_ptr<const geometry_msgs::msg::PoseArray> msg)
{
  if (_data.goal_handle == nullptr) {
    // no docking in progress --> do not process line following poses
    return;
  }

  // only process message with two poses (robot pose and target pose)
  if (msg->poses.size() != 2) {
    RCLCPP_ERROR(get_logger(), "received poses do not contain correct number of poses for line following control.");
    return;
  }

  // transform poses into robot base frame
  const auto poses_transformed = transform_poses(
    *msg, _parameter.target_frame_id, _parameter.sensor_frame_id, 
    *_tf_buffer, get_logger()
  );

  // start processing
  // take goal handle copy to prevent that it is deleted meanwhile
  auto goal_handle = _data.goal_handle;

  if (goal_handle == nullptr) {
    // error occurred meanwhile and goal handle is no longer valid
    return;
  }
  if (goal_handle->is_canceling()) {
    // goal was canceled --> stop processing
    RCLCPP_INFO(get_logger(), "goal canceled. Stop docking.");

    // stop robot and reset data
    geometry_msgs::msg::Twist twist_out;
    _pub_twist->publish(twist_out);

    auto result = std::make_shared<edu_fleet::action::TritonDocking::Result>();
    result->succeeded = false;
    goal_handle->canceled(result);
    cancelDocking();
    disableLineFollowingMode(_data.active_cluster_id);
    _data.active_cluster_id = 0;
    
    return;
  }

  const auto stamp_now = get_clock()->now();
  const double dt = std::min(0.1, (stamp_now - _data.stamp_last_processing).seconds());
  // const robot::AnglePiToPi yaw_robot = robot::algorithm::quaternion_to_yaw(poses_transformed.poses[0].orientation);
  // const robot::AnglePiToPi yaw_target = robot::algorithm::quaternion_to_yaw(poses_transformed.poses[1].orientation);

  // RCLCPP_INFO(
  //   get_logger(), "robot pose x = %f, y = %f, yaw = %f.",
  //   poses_transformed.poses[0].position.x,
  //   poses_transformed.poses[0].position.y,
  //   yaw_robot.radian()
  // );
  // RCLCPP_INFO(
  //   get_logger(), "target pose x = %f, y = %f, yaw = %f.",
  //   poses_transformed.poses[1].position.x,
  //   poses_transformed.poses[1].position.y,
  //   yaw_target.radian()
  // );

  // calculate errors
  const auto error_vector = calculate_error_vector(
    poses_transformed.poses[0], poses_transformed.poses[1]);
  const double error_x       = error_vector.x();
  const double error_y       = error_vector.y();
  const double error_heading = error_vector.z();

  // determine docking state (docking in / docking out / finished)
  determineDockingState(error_x);

  // calculate control commands
  const double vel_x    = determine_velocity_x(_data.docking_in, _data.docking_out, goal_handle->get_goal()->velocity);
  const double vel_y    = _pid_y->process(0.0, -error_y, dt);
  const double yaw_rate = _pid_heading->process(0.0, -error_heading, dt);

  // RCLCPP_INFO(get_logger(), "error x direction = %f.", error_x);
  // RCLCPP_INFO(get_logger(), "error in y direction = %f.", error_y);
  // RCLCPP_INFO(get_logger(), "velocity x = %f.", vel_x);
  // RCLCPP_INFO(get_logger(), "velocity y = %f.", vel_y);
  // RCLCPP_INFO(get_logger(), "yaw rate = %f.", yaw_rate);

  // Finish processing.
  geometry_msgs::msg::Twist twist_out;

  twist_out.linear.x = vel_x;
  twist_out.linear.y = vel_y;
  twist_out.angular.z = yaw_rate;

  _pub_twist->publish(twist_out);

  _data.stamp_last_processing = stamp_now;
}

void TritonLineFollowingController::determineDockingState(const double error_x)
{
  if (_data.docking_in == true && _data.docking_out == false && _data.at_endposition == false) {
    // docking in
    if (std::abs(error_x) >= _parameter.docking_end_error) {
      RCLCPP_INFO(get_logger(), "docking in finished.");
      _data.docking_in = false;
      _data.at_endposition = true;
      _data.stamp_endposition_reached = get_clock()->now();
    }
  }
  else if (_data.docking_in == false && _data.docking_out == false && _data.at_endposition == true) {
    // reached endposition, waiting before docking out
    const auto time_at_endposition = get_clock()->now() - _data.stamp_endposition_reached;
    if (time_at_endposition >= rclcpp::Duration(_parameter.stop_time)) {
      RCLCPP_INFO(get_logger(), "starting docking out.");
      _data.docking_out = true;
    }
  }
  else if (_data.docking_in == false && _data.docking_out == true && _data.at_endposition == true) {
    // at endposition, docking out started
    if (std::abs(error_x) < 0.03) { // \todo replace magic number
      RCLCPP_INFO(get_logger(), "left endposition.");
      _data.at_endposition = false;
    }
  }
  else if (_data.docking_in == false && _data.docking_out == true && _data.at_endposition == false) {
    // docking out
    if (std::abs(error_x) >= _parameter.docking_end_error) {
      // docking out finished --> disable line following mode
      RCLCPP_INFO(get_logger(), "docking out finished.");
      _data.docking_out = false;

      if (_data.goal_handle != nullptr) {
        auto result = std::make_shared<edu_fleet::action::TritonDocking::Result>();
        result->succeeded = true;
        _data.goal_handle->succeed(result);
      }

      disableLineFollowingMode(_data.active_cluster_id);
      _data.active_cluster_id = 0;
    }
  }

  // send feedback if goal handle is valid
  if (_data.goal_handle != nullptr) {
    auto feedback = std::make_shared<edu_fleet::action::TritonDocking::Feedback>();

    if (_data.docking_in == true && _data.docking_out == false && _data.at_endposition == false) {
      feedback->state = edu_fleet::action::TritonDocking::Feedback::DOCKING_IN;
    }
    else if (_data.docking_in == false && _data.docking_out == false && _data.at_endposition == true) {
      feedback->state = edu_fleet::action::TritonDocking::Feedback::END_POSITION;
    } 
    else if (_data.docking_in == false && _data.docking_out == true && _data.at_endposition == false) {
      feedback->state = edu_fleet::action::TritonDocking::Feedback::DOCKING_OUT;
    }
    else {
      feedback->state = edu_fleet::action::TritonDocking::Feedback::NONE;
    }
    
    _data.goal_handle->publish_feedback(feedback);
  }
}

void TritonLineFollowingController::cancelDocking()
{
  if (_data.goal_handle != nullptr) {
    auto result = std::make_shared<edu_fleet::action::TritonDocking::Result>();
    result->succeeded = false;
    _data.goal_handle->canceled(result);
    RCLCPP_INFO(get_logger(), "docking goal canceled.");
  }

  _data.docking_in = false;
  _data.docking_out = false;
  _data.at_endposition = false;
  _data.goal_handle = nullptr;
}

void TritonLineFollowingController::enableLineFollowingMode(const std::uint8_t cluster_id)
{
  if (_client_set_line_following->wait_for_service(std::chrono::seconds(1)) == false  ) {
    RCLCPP_WARN(get_logger(), "Accerion set_line_following service is not available. --> cancel docking");
    cancelDocking();
    return;
  }

  // enable line following mode for given cluster id
  auto request = std::make_shared<accerion_driver_msgs::srv::SetClusterMode::Request>();
  request->cluster_id = cluster_id;
  request->command = true;

  auto result_future = _client_set_line_following->async_send_request(
    request, [this, cluster_id](rclcpp::Client<accerion_driver_msgs::srv::SetClusterMode>::SharedFuture future) {
      auto response = future.get();
      if (response->success == false) {
        RCLCPP_ERROR(get_logger(), "could not enable line following mode for cluster id %u. --> cancel docking", cluster_id);
        cancelDocking();
      }
      else {
        RCLCPP_INFO(get_logger(), "enabled line following mode for cluster id %u.", cluster_id);
      }
  });
}

void TritonLineFollowingController::disableLineFollowingMode(const std::uint8_t cluster_id)
{
  if (_client_set_line_following->wait_for_service(std::chrono::seconds(1)) == false  ) {
    RCLCPP_WARN(get_logger(), "Accerion set_line_following service is not available. --> nothing");
    cancelDocking();
    return;
  }

  // disable line following mode for given cluster id
  auto request = std::make_shared<accerion_driver_msgs::srv::SetClusterMode::Request>();
  request->cluster_id = cluster_id;
  request->command = false;

  auto result_future = _client_set_line_following->async_send_request(
    request, [this, cluster_id](rclcpp::Client<accerion_driver_msgs::srv::SetClusterMode>::SharedFuture future) {
      auto response = future.get();
      if (response->success == false) {
        RCLCPP_ERROR(get_logger(), "could not disable line following mode for cluster id %u. --> nothing", cluster_id);
        cancelDocking();
      }
      else {
        RCLCPP_INFO(get_logger(), "disabled line following mode for cluster id %u.", cluster_id);
      }
  });
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
