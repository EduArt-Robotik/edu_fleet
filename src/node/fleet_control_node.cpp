#include "fleet_control_node.hpp"

#include <cstddef>
#include <functional>
#include <string>

namespace eduart {
namespace fleet {

static Eigen::Matrix3d calculate_fleet_to_robot_matrix(const double d_x, const double d_y, const double d_yaw)
{
  const double cos_yaw = std::cos(d_yaw);
  const double sin_yaw = std::sin(d_yaw);
  Eigen::Matrix3d t_fleet_to_robot;

  // Create transformation matrix using flexible kinematic approach.
  // Paper: Hierarchical Multi-Robot Fleet Architecture with a Kinematics-adaptive Drive System (not accepted yet...)
  t_fleet_to_robot << cos_yaw, -sin_yaw, -d_y * cos_yaw - d_x * sin_yaw,
                      sin_yaw,  cos_yaw,  d_x * cos_yaw - d_y * sin_yaw,
                          0.0,      0.0,                            1.0;

  return  t_fleet_to_robot;                          
}

FleetControlNode::Parameter FleetControlNode::get_parameter(rclcpp::Node& ros_node)
{
  FleetControlNode::Parameter parameter;

  ros_node.declare_parameter<int>("number_of_robots", parameter.number_of_robots);
  parameter.number_of_robots = ros_node.get_parameter("number_of_robots").as_int();

  // Depending on the number of robots n transformation matrices will be constructed.
  for (std::size_t i = 0; i < parameter.number_of_robots; ++i) {
    const std::string parameter_name = std::string("robot_") + std::to_string(i) + ".";
    FleetControlNode::Parameter::Offset robot_offset;

    ros_node.declare_parameter<double>(parameter_name + "d_x", robot_offset.d_x);
    robot_offset.d_x = ros_node.get_parameter(parameter_name + "d_x").as_double();

    ros_node.declare_parameter<double>(parameter_name + "d_y", robot_offset.d_y);
    robot_offset.d_y = ros_node.get_parameter(parameter_name + "d_y").as_double();

    ros_node.declare_parameter<double>(parameter_name + "d_yaw", robot_offset.d_yaw);
    robot_offset.d_yaw = ros_node.get_parameter(parameter_name + "d_yaw").as_double();   

    parameter.robot_offset.emplace_back(robot_offset);     
  }

  return parameter;
}

FleetControlNode::FleetControlNode()
  : rclcpp::Node("fleet_control_node")
  , _parameter(get_parameter(*this))
{
  for (std::size_t i = 0; i < _parameter.number_of_robots; ++i) {
    _pub_twist_robot.emplace_back(create_publisher<geometry_msgs::msg::Twist>(
      std::string("robot_") + std::to_string(i) + "/cmd_vel",
      rclcpp::QoS(1).reliable()
    ));
    _t_fleet_to_robot.emplace_back(calculate_fleet_to_robot_matrix(
      _parameter.robot_offset[i].d_x, _parameter.robot_offset[i].d_y, _parameter.robot_offset[i].d_yaw
    ));
  }
  _sub_twist_fleet = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(1).best_effort(),
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