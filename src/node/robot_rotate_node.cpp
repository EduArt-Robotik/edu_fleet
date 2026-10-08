#include "robot_rotate_node.hpp"

#include <edu_robot/algorithm/rotation.hpp>

namespace eduart {
namespace fleet {

static void publish_velocity(
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> pub, const float yaw_rate)
{
  geometry_msgs::msg::Twist cmd_vel;

  cmd_vel.angular.z = yaw_rate;
  pub->publish(cmd_vel);
}

static float limit_yaw_rate(const float yaw_rate, const float min_yaw_rate)
{
  if (yaw_rate > 0.0f) {
    return std::max(yaw_rate, min_yaw_rate);
  } else if (yaw_rate < 0.0f) {
    return std::min(yaw_rate, -min_yaw_rate);
  }

  return 0.0f;
}

RobotRotateNode::Parameter RobotRotateNode::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<int>(
    "process_interval_ms", static_cast<int>(default_parameter.process_interval.count()));
  ros_node.declare_parameter<double>(
    "yaw_error_tolerance_deg", default_parameter.yaw_error_tolerance.degree());
  ros_node.declare_parameter<float>("min_yaw_rate", default_parameter.min_yaw_rate);
  ros_node.declare_parameter<float>("kp", default_parameter.kp);

  parameter.process_interval = std::chrono::milliseconds(
    ros_node.get_parameter("process_interval_ms").as_int());
  parameter.yaw_error_tolerance = robot::Angle::createFromDegree(
    ros_node.get_parameter("yaw_error_tolerance_deg").as_double());
  parameter.min_yaw_rate = ros_node.get_parameter("min_yaw_rate").as_double();
  parameter.kp = ros_node.get_parameter("kp").as_double();

  return parameter;
}

RobotRotateNode::RobotRotateNode()
  : rclcpp::Node("robot_turn_node")
  , _parameter(get_parameter({}, *this))
{
  _sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
    "in/odometry", rclcpp::QoS(2).best_effort(), 
    std::bind(&RobotRotateNode::callbackOdometry, this, std::placeholders::_1)
  );
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/cmd_vel", rclcpp::QoS(2).reliable()
  );
  _action_server = rclcpp_action::create_server<edu_fleet::action::RobotRotate>(
    this,
    "robot_rotate",
    std::bind(&RobotRotateNode::callbackAcceptGoal, this, std::placeholders::_1, std::placeholders::_2),
    [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle) {
      // accept all cancel requests
      (void)goal_handle;

      RCLCPP_INFO(get_logger(), "received goal cancel request");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&RobotRotateNode::callbackGoal, this, std::placeholders::_1)
  );

  // PID controller shall be in range of -1.0 to 1.0 == 100%
  _pid_controller = std::make_shared<fleet::controller::Pid>(fleet::controller::Pid::Parameter{
    _parameter.kp, 0.0f, 0.0f, 1.0, 1.0f, true
  });
}

RobotRotateNode::~RobotRotateNode()
{

}

void RobotRotateNode::callbackOdometry(std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
  std::lock_guard<std::mutex> lock(_data.mutex);
  _data.yaw = robot::algorithm::quaternion_to_yaw(msg->pose.pose.orientation);
}

rclcpp_action::GoalResponse RobotRotateNode:: callbackAcceptGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const edu_fleet::action::RobotRotate::Goal> goal)
{
  (void)uuid;

  // only accept action if no already executing
  if (_data.is_executing || _data.goal_is_accepted) {
    RCLCPP_WARN(get_logger(), "already accepted goal and action is in execution, rejecting new goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // accept goal
  _data.goal_is_accepted = true;
  _data.start_yaw = _data.yaw;
  _data.end_yaw = _data.start_yaw + robot::AnglePiToPi(goal->relative_yaw);
  _pid_controller->reset();
  
  RCLCPP_INFO(get_logger(), "received goal request, accepting and executing. Robot will rotate by %.2f radian", goal->relative_yaw);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void RobotRotateNode::callbackGoal(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle)
{
  // \todo don't like this. Detached thread smells bad. Thread needs to be finished when node is shutting down!
  std::thread{std::bind(&RobotRotateNode::executeRotate, this, std::placeholders::_1), goal_handle}.detach();
}

void RobotRotateNode::executeRotate(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle)
{
  _data.is_executing = true;
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "rotating robot by %.2f radian", goal->relative_yaw);
  rclcpp::Rate loop_rate(rclcpp::Duration(_parameter.process_interval));
  auto feedback = std::make_shared<edu_fleet::action::RobotRotate::Feedback>();
  auto result = std::make_shared<edu_fleet::action::RobotRotate::Result>();

  // Integrate actual driven angle from odometry to prevent overshoot
  robot::AnglePiToPi current_yaw;
  {
    std::lock_guard<std::mutex> lock(_data.mutex);
    current_yaw = _data.yaw;
  }
  float integrated_yaw = 0.0f; // [rad], accumulated driven yaw since start of rotation
  const float target_relative_yaw = std::abs(goal->relative_yaw); 

  // loop until goal is reached or canceled (also function must finish when node is shutting down!)
  while (rclcpp::ok()) {
    // integrate driven yaw (integral is positive always!)
    {
      std::lock_guard<std::mutex> lock(_data.mutex);
      integrated_yaw += std::abs(robot::AnglePiToPi(_data.yaw - current_yaw));
      current_yaw = _data.yaw;
    }

    if (goal_handle->is_canceling()) {
      // goal was canceled --> stop rotation
      std::lock_guard<std::mutex> lock(_data.mutex);
      result->current_yaw = _data.yaw;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "goal canceled. Stop rotation at current yaw %.2f", _data.yaw.radian());

      // stop robot --> reset data
      publish_velocity(_pub_velocity, 0.0f);
      _data.is_executing = false;
      _data.goal_is_accepted = false;
      return; // exit function, don't call succeed()
    }

    // check if goal is reached, negative error means goal reached
    const float yaw_error = target_relative_yaw - integrated_yaw;

    if (yaw_error <= _parameter.yaw_error_tolerance.radian()) {
      result->error_yaw = yaw_error;
      RCLCPP_INFO(get_logger(), "goal succeeded. Reached target relative yaw %.2f with error %.2f (integrated %.2f)",
        target_relative_yaw, std::abs(yaw_error), integrated_yaw);
      break;
    }

    // precessing data
    const float control_variable = _pid_controller->process(
      0.0f, yaw_error, static_cast<float>(_parameter.process_interval.count()) / 1000.0f
    );
    const float yaw_rate = std::abs(control_variable) * goal->yaw_rate; // goal yaw rate sets the rotation direction
    publish_velocity(_pub_velocity, limit_yaw_rate(yaw_rate, _parameter.min_yaw_rate));

    // sleep to get correct loop rate
    loop_rate.sleep();
  }

  // goal reached --> stop rotation
  std::lock_guard<std::mutex> lock(_data.mutex);
  result->current_yaw = _data.yaw;

  goal_handle->succeed(result);
  publish_velocity(_pub_velocity, 0.0f);
  _data.is_executing = false;
  _data.goal_is_accepted = false;
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::RobotRotateNode>());
  rclcpp::shutdown();

  return 0;
}
