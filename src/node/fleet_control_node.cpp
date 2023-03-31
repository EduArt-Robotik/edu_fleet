#include "fleet_control_node.hpp"

#include <cstddef>
#include <functional>
#include <string>

namespace eduart {
namespace fleet {

using namespace std::chrono_literals;

static Eigen::Matrix3d calculate_fleet_to_robot_matrix(const double d_x, const double d_y, const double d_yaw)
{
  const double cos_yaw = std::cos(d_yaw);
  const double sin_yaw = std::sin(d_yaw);
  Eigen::Matrix3d t_fleet_to_robot;

  // Create transformation matrix using flexible kinematic approach.
  // Paper: Hierarchical Multi-Robot Fleet Architecture with a Kinematics-adaptive Drive System
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
    FleetControlNode::Parameter::Pose2D robot_pose;

    ros_node.declare_parameter<double>(parameter_name + "x", robot_pose.x);
    robot_pose.x = ros_node.get_parameter(parameter_name + "x").as_double();

    ros_node.declare_parameter<double>(parameter_name + "y", robot_pose.y);
    robot_pose.y = ros_node.get_parameter(parameter_name + "y").as_double();

    ros_node.declare_parameter<double>(parameter_name + "yaw", robot_pose.yaw);
    robot_pose.yaw = ros_node.get_parameter(parameter_name + "yaw").as_double();   

    parameter.robot_pose.emplace_back(robot_pose);     
  }

  return parameter;
}

FleetControlNode::FleetControlNode()
  : rclcpp::Node("fleet_control_node")
  , _parameter(get_parameter(*this))
{
  for (std::size_t i = 0; i < _parameter.number_of_robots; ++i) {
    const std::string robot_namespace = std::string("robot_") + std::to_string(i);
    _pub_twist_robot.emplace_back(create_publisher<geometry_msgs::msg::Twist>(
      robot_namespace + "/cmd_vel",
      rclcpp::QoS(1).reliable()
    ));
    _srv_client_get_kinematic.emplace_back(create_client<edu_robot::srv::GetKinematicDescription>(
      robot_namespace + "/get_kinematic_description"
    ));

    // Initialize twist calculation variables with default values.
    _t_fleet_to_robot.emplace_back(calculate_fleet_to_robot_matrix(
      _parameter.robot_pose[i].x, _parameter.robot_pose[i].y, _parameter.robot_pose[i].yaw
    ));
    _kinematic_matrix.emplace_back(Eigen::Matrix3f::Identity());
    _robot_rpm_limit.emplace_back();
  }
  _sub_twist_fleet = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(1).best_effort(),
    std::bind(&FleetControlNode::callbackTwistFleet, this, std::placeholders::_1)
  );
  _timer_update_kinematic = create_wall_timer(2s, std::bind(&FleetControlNode::updateKinematicDescription, this));
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

void FleetControlNode::updateKinematicDescription()
{
  for (std::size_t i = 0; i < _srv_client_get_kinematic.size(); ++i) {
    auto request = std::make_shared<edu_robot::srv::GetKinematicDescription::Request>();
    auto& client = _srv_client_get_kinematic[i];

    if (client->service_is_ready()) {
      RCLCPP_INFO(get_logger(), "Requesting kinematic description on: %s", client->get_service_name());
      client->async_send_request(
        request,
        [this, i] (rclcpp::Client<edu_robot::srv::GetKinematicDescription>::SharedFutureWithRequest future) {
          processKinematicDescription(future.get().second->kinematic, i);
        }
      );
    }
    // else:
    //  do nothing
  }
}

void FleetControlNode::processKinematicDescription(
  const edu_robot::msg::RobotKinematicDescription& description, const std::size_t robot_index)
{
  if (robot_index >= _kinematic_matrix.size() || robot_index >= _robot_rpm_limit.size()) {
    RCLCPP_ERROR(get_logger(), "Robot index out of range. Must not be happen! Debug it!");
    return;
  }

  _kinematic_matrix[robot_index].resize(description.k.rows, description.k.cols);
  _robot_rpm_limit[robot_index].clear();
  _robot_rpm_limit[robot_index].reserve(description.wheel_limits.size());

  for (Eigen::Index row = 0; row < _kinematic_matrix[robot_index].rows(); ++row) {
    for (Eigen::Index col = 0; col < _kinematic_matrix[robot_index].cols(); ++col) {
      _kinematic_matrix[robot_index](row, col) = description.k.data[row * description.k.cols + description.k.cols];
    }
  }
  for (const auto& limit : description.wheel_limits) {
    _robot_rpm_limit[robot_index].emplace_back(limit);
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