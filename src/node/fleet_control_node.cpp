#include "fleet_control_node.hpp"

#include <cstddef>
#include <functional>
#include <string>

namespace eduart {
namespace fleet {

FleetControlNode::Parameter FleetControlNode::get_parameter(rclcpp::Node& ros_node)
{
  FleetControlNode::Parameter parameter;

  ros_node.declare_parameter<int>("number_of_robots", parameter.number_of_robots);

  parameter.number_of_robots = ros_node.get_parameter("number_of_robots").as_int();

  return parameter;
}

FleetControlNode::FleetControlNode()
  : rclcpp::Node("fleet_control_node")
  , _parameter(get_parameter(*this))
{
  for (std::size_t i = 0; i < _parameter.number_of_robots; ++i) {
    _pub_twist_robot[i] = create_publisher<geometry_msgs::msg::Twist>(
      std::string("robot_") + std::to_string(i) + "/cmd_vel",
      rclcpp::QoS(1).reliable()
    );
  }
  _sub_twist_fleet = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(1).reliable(),
    std::bind(&FleetControlNode::callbackTwistFleet, this, std::placeholders::_1)
  );
  // \todo request kinematic matrices.
}

FleetControlNode::~FleetControlNode()
{

}

static geometry_msgs::msg::Twist to_twist_message(const Eigen::Vector3d& velocity)
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = velocity.x();
  msg.linear.y = velocity.y();
  msg.linear.z = 0.0;

  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = velocity.z();

  return msg;
}

void FleetControlNode::callbackTwistFleet(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg)
{
  const Eigen::Vector3d velocity(twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);

  for (std::size_t i = 0; i < _t_fleet_to_robot.size(); ++i) {
    const Eigen::Vector3d velocity_robot = _t_fleet_to_robot[i] * velocity;
    // \todo handle maximum rpm limit of the robot's wheels.
    _pub_twist_robot[i]->publish(to_twist_message(velocity_robot));
  }
}

} // end namespace fleet
} // end namespace eduart


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<eduart::fleet::FleetControlNode>());
  rclcpp::shutdown();

  return 0;
}