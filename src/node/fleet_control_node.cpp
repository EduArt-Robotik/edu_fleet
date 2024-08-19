#include "fleet_control_node.hpp"

#include <edu_fleet/sensor_model/message_converting.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
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
  , _robot(_parameter.number_of_robots)
{
  for (std::size_t i = 0; i < _robot.size(); ++i) {
    const std::string robot_namespace = std::string("robot_") + std::to_string(i);

    // create publisher
    _robot[i].pub_twist_robot = create_publisher<geometry_msgs::msg::Twist>(
      robot_namespace + "/cmd_vel",
      rclcpp::QoS(2).reliable()
    );
    _robot[i].pub_set_lighting = create_publisher<edu_robot::msg::SetLightingColor>(
      robot_namespace + "/set_lighting_color",
      rclcpp::QoS(2).reliable()
    );
    _robot[i].pub_target_pose = create_publisher<geometry_msgs::msg::PoseStamped>(
      robot_namespace + "/target_pose",
      rclcpp::QoS(2).reliable()
    );

    // create subscriptions
    _robot[i].sub_kinematic_description = create_subscription<edu_robot::msg::RobotKinematicDescription>(
      robot_namespace + "/kinematic_description",
      rclcpp::QoS(2).reliable().transient_local(),
      [this, i](std::shared_ptr<const edu_robot::msg::RobotKinematicDescription> description) {
        processKinematicDescription(description, i);
      }
    );
    _robot[i].sub_robot_status = create_subscription<edu_robot::msg::RobotStatusReport>(
      robot_namespace + "/status_report",
      rclcpp::QoS(2).best_effort().durability_volatile(),
      [this, i](std::shared_ptr<const edu_robot::msg::RobotStatusReport> report) {
        callbackRobotStatusReport(report, i);
      }
    );
    _robot[i].sub_localization = create_subscription<nav_msgs::msg::Odometry>(
      robot_namespace + "/localization",
      rclcpp::QoS(2).best_effort(),
      [this, i](std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
        callbackLocalization(msg, i);
      }
    );

    // Initialize twist calculation variables with default values.
    _robot[i].t_fleet_to_robot_velocity = calculate_fleet_to_robot_velocity_matrix(
      _parameter.robot_pose[i].x, _parameter.robot_pose[i].y, _parameter.robot_pose[i].yaw
    );
    _robot[i].t_fleet_to_robot_transform = calculate_fleet_to_robot_transform_matrix(
      _parameter.robot_pose[i].x, _parameter.robot_pose[i].y, _parameter.robot_pose[i].yaw
    );
    _robot[i].kinematic_matrix = Eigen::Matrix3d::Identity();
    _robot[i].lost_fleet_formation = 0;
    _robot[i].current_mode = edu_robot::msg::Mode::INACTIVE;
    _robot[i].position = Eigen::Vector2d::Zero();
    _robot[i].orientation = 0.0;
  }

  _sub_twist_fleet = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(1).best_effort(),
    std::bind(&FleetControlNode::callbackTwistFleet, this, std::placeholders::_1)
  );
  _srv_server_get_transform = create_service<edu_fleet::srv::GetTransform>(
    "/get_transform",
    std::bind(&FleetControlNode::callbackServiceGetTransform, this, std::placeholders::_1, std::placeholders::_2)
  );
}

FleetControlNode::~FleetControlNode()
{

}

void FleetControlNode::callbackTwistFleet(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg)
{
  using sensor_model::message_converting;

  // Calculate each robot's velocity.
  const Eigen::Vector3d velocity(twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> velocity_robot(_parameter.number_of_robots);

  // Keep robot's wheel speed below physical limits.
  double reduce_factor = 1.0;

  for (std::size_t robot_idx = 0; robot_idx < velocity_robot.size(); ++robot_idx) {
    velocity_robot[robot_idx] = _robot[robot_idx].t_fleet_to_robot_velocity * velocity;

    // Calculate wheel rotation speed using provided kinematic matrix.
    // Apply velocity reduction if a limit is reached.
    Eigen::VectorXd radps = _robot[robot_idx].kinematic_matrix * velocity_robot[robot_idx];

    // Calculate reduce factor if one or more motors are over limit.
    for (std::size_t wheel_idx = 0; wheel_idx < _robot[robot_idx].rpm_limit.size(); ++wheel_idx) {
      reduce_factor = std::min<double>(
        std::abs(_robot[robot_idx].rpm_limit[wheel_idx] / eduart::robot::Rpm::fromRadps(radps(wheel_idx))),
        reduce_factor
      );
    }
  }
  // If fleet formation is lost stop fleet movement using reduce factor to get back formation.
  for (std::size_t robot_idx = 0; robot_idx < _robot.size(); ++robot_idx) {
    reduce_factor = std::min<double>(1.0f - static_cast<float>(_robot[robot_idx].lost_fleet_formation) / 100.0f, reduce_factor);
  }

  // calculate new fleet pose
  const double dt = static_cast<double>(_parameter.expected_sending_interval.count()) / 1000.0;
  const Eigen::Vector2d v(twist_msg->linear.x, twist_msg->linear.y);
  
  _fleet_position += reduce_factor * dt * v;
  _fleet_orientation += dt * robot::AnglePiToPi(twist_msg->angular.z);

  // calculate new robot poses
  for (std::size_t robot_idx = 0; robot_idx < _robot.size(); ++robot_idx) {
    const Eigen::Vector2d v(velocity_robot[robot_idx].x(), velocity_robot[robot_idx].y());
    _robot[robot_idx].target_position = _robot[robot_idx].position + reduce_factor * dt * v;
    _robot[robot_idx].target_orientation = _robot[robot_idx].orientation + reduce_factor + dt * velocity_robot[robot_idx].z();
  }

  // publishing new control commands and set points
  for (std::size_t robot_idx = 0; robot_idx < velocity_robot.size(); ++robot_idx) {
    // velocity feed forward control
    _robot[robot_idx].pub_twist_robot->publish(message_converting<>::to_ros(velocity_robot[robot_idx] * reduce_factor));

    // target pose for pose controller on the robot
    geometry_msgs::msg::PoseStamped pose;
    pose.header =  
    _robot[robot_idx].pub_target_pose->publish()
  }
}

// void FleetControlNode::callbackTwistDriftCompensation(
//   std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg, const std::size_t robot_index)
// {
//   if (_current_robot_mode[robot_index] != edu_robot::msg::Mode::AUTONOMOUS) {
//     return;
//   }

//   const Eigen::Vector2d twist_cmd(twist_msg->linear.x, twist_msg->linear.y);
//   const double velocity = twist_cmd.norm();

//   if (_lost_fleet_formation[robot_index] == 0 && velocity >= _parameter.drift_limit) {
//     RCLCPP_INFO(get_logger(), "Robot %lu lost fleet formation.", robot_index);
//     // Lost fleet formation!
//     _lost_fleet_formation[robot_index] = 100; // == 100%

//     // Indicate lost by flashing red lighting.
//     edu_robot::msg::SetLightingColor lighting_msg;

//     lighting_msg.r = 25;
//     lighting_msg.g = 0;
//     lighting_msg.b = 0;

//     lighting_msg.brightness.data = 0.7;
//     lighting_msg.lighting_name = "all";
//     lighting_msg.mode = edu_robot::msg::SetLightingColor::FLASH;

//     _pub_set_lighting[robot_index]->publish(lighting_msg);
//   }
//   else if (_lost_fleet_formation[robot_index] != 0 && velocity < 0.02) {
//     RCLCPP_INFO(get_logger(), "Robot %lu back in fleet formation.", robot_index);    
//     // Fleet formation is fine.
//     _lost_fleet_formation[robot_index] = 0; // == 0%

//     // Indicate normal operation by flashing white lighting.
//     edu_robot::msg::SetLightingColor lighting_msg;

//     lighting_msg.r = 25;
//     lighting_msg.g = 25;
//     lighting_msg.b = 25;

//     lighting_msg.brightness.data = 0.7;
//     lighting_msg.lighting_name = "all";
//     lighting_msg.mode = edu_robot::msg::SetLightingColor::FLASH;

//     _pub_set_lighting[robot_index]->publish(lighting_msg);    
//   }
//   else if (_lost_fleet_formation[robot_index] != 0) {
//     // Calculate new formation lost indicator value.
//     _lost_fleet_formation[robot_index] = static_cast<std::uint8_t>((velocity / _parameter.drift_limit) * 100.0);
//     _lost_fleet_formation[robot_index] = std::min<std::uint8_t>(_lost_fleet_formation[robot_index], 100);
//   }
// }

void FleetControlNode::callbackRobotStatusReport(
    std::shared_ptr<const edu_robot::msg::RobotStatusReport> report, const std::size_t robot_index)
{
  _robot[robot_index].current_mode = report->robot_state.mode.mode;
}

void FleetControlNode::callbackLocalization(
  std::shared_ptr<const nav_msgs::msg::Odometry> msg, const std::size_t robot_index)
{
  _robot[robot_index].position.x() = msg->pose.pose.position.x;
  _robot[robot_index].position.y() = msg->pose.pose.position.y;

  _robot[robot_index].orientation = sensor_model::message_converting<>::quaternion_to_yaw(
    msg->pose.pose.orientation
  );
}

void FleetControlNode::callbackServiceGetTransform(
  const std::shared_ptr<edu_fleet::srv::GetTransform::Request> request, 
  std::shared_ptr<edu_fleet::srv::GetTransform::Response> response)
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
  const auto t_from = _robot[idx_from].t_fleet_to_robot_transform;
  const auto t_to = _robot[idx_to].t_fleet_to_robot_transform;
  const Eigen::Matrix3d t = t_from.inverse() * t_to;

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
  if (static_cast<Eigen::Index>(robot_index) >= _robot[robot_index].kinematic_matrix.size() || robot_index >= _robot.size()) {
    RCLCPP_ERROR(get_logger(), "Robot index out of range. Must not be happen! Debug it!");
    return;
  }

  _robot[robot_index].kinematic_matrix.resize(description->k.rows, description->k.cols);
  _robot[robot_index].rpm_limit.clear();
  _robot[robot_index].rpm_limit.reserve(description->wheel_limits.size());

  for (Eigen::Index row = 0; row < _robot[robot_index].kinematic_matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < _robot[robot_index].kinematic_matrix.cols(); ++col) {
      _robot[robot_index].kinematic_matrix(row, col) = description->k.data[row * description->k.cols + col];
    }
  }
  for (const auto& limit : description->wheel_limits) {
    _robot[robot_index].rpm_limit.emplace_back(limit);
  }
  RCLCPP_INFO_STREAM(get_logger(), "received kinematic matrix:\n" << _robot[robot_index].kinematic_matrix);
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