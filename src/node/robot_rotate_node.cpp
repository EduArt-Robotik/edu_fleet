#include "robot_rotate_node.hpp"

namespace eduart {
namespace fleet {

RobotRotateNode::RobotRotateNode()
  : rclcpp::Node("robot_turn_node")
{
  _sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
    "in/odometry", rclcpp::QoS(2).best_effort(), 
    std::bind(&RobotRotateNode::callbackOdometry, this, std::placeholders::_1)
  );

  _action_server = rclcpp_action::create_server<edu_fleet::action::RobotRotate>(
    this,
    "robot_rotate",
    std::bind(&RobotRotateNode::callbackAcceptGoal, this, std::placeholders::_1, std::placeholders::_2),
    [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle) {
      (void)goal_handle;

      RCLCPP_INFO(get_logger(), "Received goal cancel request");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    std::bind(&RobotRotateNode::callbackGoal, this, std::placeholders::_1)
  );
}

RobotRotateNode::~RobotRotateNode()
{

}

void RobotRotateNode::callbackOdometry(std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "odometry message received");
}

rclcpp_action::GoalResponse RobotRotateNode::callbackAcceptGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const edu_fleet::action::RobotRotate::Goal> goal)
{
  (void)uuid;
  (void)goal;

  RCLCPP_INFO(get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void RobotRotateNode::callbackGoal(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle)
{
  std::thread{std::bind(&RobotRotateNode::executeRotate, this, std::placeholders::_1), goal_handle}.detach();
}

void RobotRotateNode::executeRotate(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<edu_fleet::action::RobotRotate>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(get_logger(), "rotating robot by %.2f radian", goal->relative_yaw);
  rclcpp::Rate loop_rate(10);
  auto feedback = std::make_shared<edu_fleet::action::RobotRotate::Feedback>();
  auto result = std::make_shared<edu_fleet::action::RobotRotate::Result>();

  

  for (int i = 1; (i <= 10) && rclcpp::ok(); ++i) {
    if (goal_handle->is_canceling()) {
      result->total_angle_rotated = feedback->partial_angle_rotated;
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Goal canceled");
      return;
    }

    feedback->partial_angle_rotated = i * (goal->angle / 10.0);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Published feedback: %.2f", feedback->partial_angle_rotated);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->total_angle_rotated = feedback->partial_angle_rotated;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  } else {
    RCLCPP_INFO(get_logger(), "Goal aborted");
  }
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
