#include "fleet_control_node.hpp"

#include <Eigen/Geometry>

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <functional>
#include <iterator>
#include <string>

namespace eduart {
namespace fleet {

using namespace std::chrono_literals;

static Eigen::Matrix3d calculate_fleet_to_robot_velocity_matrix(const double d_x, const double d_y, const double d_yaw)
{
  const double cos_yaw = std::cos(-d_yaw);
  const double sin_yaw = std::sin(-d_yaw);
  Eigen::Matrix3d t_fleet_to_robot;

  // Create transformation matrix using flexible kinematic approach.
  // Paper: Hierarchical Multi-Robot Fleet Architecture with a Kinematics-adaptive Drive System
  t_fleet_to_robot << cos_yaw, -sin_yaw, -d_y * cos_yaw - d_x * sin_yaw,
                      sin_yaw,  cos_yaw,  d_x * cos_yaw - d_y * sin_yaw,
                          0.0,      0.0,                            1.0;

  return  t_fleet_to_robot;                          
}

static Eigen::Matrix3d calculate_fleet_to_robot_transform_matrix(const double x, const double y, const double yaw)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  Eigen::Matrix3d t_fleet_to_robot;

  t_fleet_to_robot << cos_yaw, -sin_yaw,   x,
                      sin_yaw,  cos_yaw,   y,
                          0.0,      0.0, 1.0;

  return  t_fleet_to_robot;      
}

FleetControlNode::Parameter FleetControlNode::get_parameter(rclcpp::Node& ros_node)
{
  FleetControlNode::Parameter parameter;

  ros_node.declare_parameter<int>("number_of_robots", parameter.number_of_robots);
  ros_node.declare_parameter<double>("drift_limit", parameter.drift_limit);
  parameter.number_of_robots = ros_node.get_parameter("number_of_robots").as_int();
  parameter.drift_limit = ros_node.get_parameter("drift_limit").as_double();

  // Depending on the number of robots n transformation matrices will be constructed.
  for (std::size_t i = 0; i < parameter.number_of_robots; ++i) {
    const std::string robot_name = std::string("robot_") + std::to_string(i);
    FleetControlNode::Parameter::Pose2D robot_pose;

    ros_node.declare_parameter<double>(robot_name + ".x", robot_pose.x);
    ros_node.declare_parameter<double>(robot_name + ".y", robot_pose.y);
    ros_node.declare_parameter<double>(robot_name + ".yaw", robot_pose.yaw);
    ros_node.declare_parameter<std::string>(robot_name + ".name", robot_name);

    robot_pose.x = ros_node.get_parameter(robot_name + ".x").as_double();
    robot_pose.y = ros_node.get_parameter(robot_name + ".y").as_double();
    robot_pose.yaw = ros_node.get_parameter(robot_name + ".yaw").as_double();
    parameter.robot_name.emplace_back(ros_node.get_parameter(robot_name + ".name").as_string());
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
    _pub_set_lighting.emplace_back(create_publisher<edu_robot::msg::SetLightingColor>(
      robot_namespace + "/set_lighting_color",
      rclcpp::QoS(1).reliable()
    ));
    _sub_kinematic_description.emplace_back(create_subscription<edu_robot::msg::RobotKinematicDescription>(
      robot_namespace + "/kinematic_description",
      rclcpp::QoS(2).reliable().transient_local(),
      [this, i](std::shared_ptr<const edu_robot::msg::RobotKinematicDescription> description) {
        processKinematicDescription(description, i);
      }
    ));
    _sub_twist_drift_compensation.emplace_back(create_subscription<geometry_msgs::msg::Twist>(
      robot_namespace + "/twist_compensation", 
      rclcpp::QoS(2).best_effort().durability_volatile(), 
      [this, i](std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg) {
        callbackTwistDriftCompensation(twist_msg, i);
      }
    ));
    _sub_robot_status.emplace_back(create_subscription<edu_robot::msg::RobotStatusReport>(
      robot_namespace + "/status_report",
      rclcpp::QoS(2).best_effort().durability_volatile(),
      [this, i](std::shared_ptr<const edu_robot::msg::RobotStatusReport> report) {
        callbackRobotStatusReport(report, i);
      }
    ));

    // Initialize twist calculation variables with default values.
    _t_fleet_to_robot_velocity.emplace_back(calculate_fleet_to_robot_velocity_matrix(
      _parameter.robot_pose[i].x, _parameter.robot_pose[i].y, _parameter.robot_pose[i].yaw
    ));
    _t_fleet_to_robot_transform.emplace_back(calculate_fleet_to_robot_transform_matrix(
      _parameter.robot_pose[i].x, _parameter.robot_pose[i].y, _parameter.robot_pose[i].yaw
    ));    
    _kinematic_matrix.emplace_back(Eigen::Matrix3d::Identity());
    _robot_rpm_limit.emplace_back();
    _lost_fleet_formation.push_back(false);
    _current_robot_mode.push_back(edu_robot::msg::Mode::INACTIVE);
  }

  _sub_twist_fleet = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(1).best_effort(),
    std::bind(&FleetControlNode::callbackTwistFleet, this, std::placeholders::_1)
  );
  _srv_server_get_transform = create_service<edu_swarm::srv::GetTransform>(
    "/get_transform",
    std::bind(&FleetControlNode::callbackServiceGetTransform, this, std::placeholders::_1, std::placeholders::_2)
  );
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
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> velocity_robot(_parameter.number_of_robots);
  float reduce_factor = 1.0;

  for (std::size_t robot_idx = 0; robot_idx < velocity_robot.size(); ++robot_idx) {
    velocity_robot[robot_idx] = _t_fleet_to_robot_velocity[robot_idx] * velocity;

    // Calculate wheel rotation speed using provided kinematic matrix.
    // Apply velocity reduction if a limit is reached.
    Eigen::VectorXd radps = _kinematic_matrix[robot_idx] * velocity_robot[robot_idx];

    // Calculate reduce factor if one or more motors are over limit.
    for (std::size_t wheel_idx = 0; wheel_idx < _robot_rpm_limit[robot_idx].size(); ++wheel_idx) {
      reduce_factor = std::min(
        std::abs(_robot_rpm_limit[robot_idx][wheel_idx] / eduart::robot::Rpm::fromRadps(radps(wheel_idx))),
        reduce_factor
      );
    }
  }
  // If fleet formation is lost stop fleet movement using reduce factor.
  for (const auto lost_formation : _lost_fleet_formation) {
    if (lost_formation) {
      reduce_factor = 0.0f;
    }
  }
  std::cout << "reduce factor = " << reduce_factor << std::endl;
  for (std::size_t robot_idx = 0; robot_idx < velocity_robot.size(); ++robot_idx) {
    _pub_twist_robot[robot_idx]->publish(to_twist_message(velocity_robot[robot_idx] * reduce_factor));
  }
}

void FleetControlNode::callbackTwistDriftCompensation(
  std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg, const std::size_t robot_index)
{
  if (_current_robot_mode[robot_index] != edu_robot::msg::Mode::AUTONOMOUS) {
    return;
  }

  const Eigen::Vector2d twist_cmd(twist_msg->linear.x, twist_msg->linear.y);

  // _lost_fleet_formation[robot_index] = twist_cmd.squaredNorm() >= _parameter.drift_limit * _parameter.drift_limit;

  if (_lost_fleet_formation[robot_index] == false && twist_cmd.squaredNorm() >= _parameter.drift_limit * _parameter.drift_limit) {
    RCLCPP_INFO(get_logger(), "Robot %lu lost fleet formation.", robot_index);
    // Lost fleet formation!
    _lost_fleet_formation[robot_index] = true;

    // Indicate lost by flashing red lighting.
    edu_robot::msg::SetLightingColor lighting_msg;

    lighting_msg.r = 25;
    lighting_msg.g = 0;
    lighting_msg.b = 0;

    lighting_msg.brightness.data = 0.7;
    lighting_msg.lighting_name = "all";
    lighting_msg.mode = edu_robot::msg::SetLightingColor::FLASH;

    _pub_set_lighting[robot_index]->publish(lighting_msg);
  }
  else if (_lost_fleet_formation[robot_index] == true && twist_cmd.squaredNorm() < 0.02) {
    RCLCPP_INFO(get_logger(), "Robot %lu back in fleet formation.", robot_index);    
    // Fleet formation is fine.
    _lost_fleet_formation[robot_index] = false;

    // Indicate normal operation by flashing white lighting.
    edu_robot::msg::SetLightingColor lighting_msg;

    lighting_msg.r = 25;
    lighting_msg.g = 25;
    lighting_msg.b = 25;

    lighting_msg.brightness.data = 0.7;
    lighting_msg.lighting_name = "all";
    lighting_msg.mode = edu_robot::msg::SetLightingColor::FLASH;

    _pub_set_lighting[robot_index]->publish(lighting_msg);    
  }
}

void FleetControlNode::callbackRobotStatusReport(
    std::shared_ptr<const edu_robot::msg::RobotStatusReport> report, const std::size_t robot_index)
{
  _current_robot_mode[robot_index] = report->robot_state.mode.mode;
}

void FleetControlNode::callbackServiceGetTransform(
  const std::shared_ptr<edu_swarm::srv::GetTransform::Request> request, 
  std::shared_ptr<edu_swarm::srv::GetTransform::Response> response)
{
  const auto search_from = std::find(
    _parameter.robot_name.begin(), _parameter.robot_name.end(), request->from_robot);
  const auto search_to = std::find(
    _parameter.robot_name.begin(), _parameter.robot_name.end(), request->to_robot);

  // robot to fleet, fleet to robot = transform
  if (search_from == _parameter.robot_name.end() || search_to == _parameter.robot_name.end()) {
    // throw std::runtime_error("Requested transformation is not available.");
    return;
  }

  // Calculate transformation: t_to * t_from.inv() * p = t * p
  const std::size_t idx_from = search_from - _parameter.robot_name.begin();
  const std::size_t idx_to = search_to - _parameter.robot_name.begin();
  // std::cout << "idx from = " << idx_from << std::endl;
  // std::cout << "idx to = "<< idx_to << std::endl;
  const auto t_from = _t_fleet_to_robot_transform[idx_from];
  const auto t_to = _t_fleet_to_robot_transform[idx_to];
  // std::cout << "t from:\n" << t_from << std::endl;
  // std::cout << "t to:\n" << t_to << std::endl;
  const Eigen::Matrix3d t = t_from.inverse() * t_to;
  std::cout << "t:\n" << t << std::endl;
  // std::cout << "p:\n" << t * Eigen::Vector3d(0.0, 0.0, 1.0) << std::endl;

  // Copying Result to Response
  response->t.rows = t.rows();
  response->t.cols = t.cols();
  response->t.data.resize(t.rows() * t.cols());

  for (Eigen::Index row = 0; row < t.rows(); ++row) {
    for (Eigen::Index col = 0; col < t.cols(); ++col) {
      response->t.data[row * t.cols() + col] = t(row, col);
    }
  }

  for (std::size_t i = 0; i < response->t.data.size(); ++i) {
    std::cout << response->t.data[i] << ", ";
  }
  std::cout << std::endl;
}    

void FleetControlNode::processKinematicDescription(
  std::shared_ptr<const edu_robot::msg::RobotKinematicDescription> description, const std::size_t robot_index)
{
  if (robot_index >= _kinematic_matrix.size() || robot_index >= _robot_rpm_limit.size()) {
    RCLCPP_ERROR(get_logger(), "Robot index out of range. Must not be happen! Debug it!");
    return;
  }

  _kinematic_matrix[robot_index].resize(description->k.rows, description->k.cols);
  _robot_rpm_limit[robot_index].clear();
  _robot_rpm_limit[robot_index].reserve(description->wheel_limits.size());

  for (Eigen::Index row = 0; row < _kinematic_matrix[robot_index].rows(); ++row) {
    for (Eigen::Index col = 0; col < _kinematic_matrix[robot_index].cols(); ++col) {
      _kinematic_matrix[robot_index](row, col) = description->k.data[row * description->k.cols + col];
    }
  }
  for (const auto& limit : description->wheel_limits) {
    _robot_rpm_limit[robot_index].emplace_back(limit);
  }
  std::cout << "kinematic matrix:\n" << _kinematic_matrix[robot_index] << std::endl;
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