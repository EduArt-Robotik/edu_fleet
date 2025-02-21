#include "collision_avoidance_lidar_field_node.hpp"

namespace eduart {
namespace fleet {

static void limit_velocity(double& velocity, const double limit)
{
  if (velocity >= 0.0) {
    velocity = std::min(velocity, limit);
  }
  else {
    velocity = std::max(velocity, -limit);
  }
}

CollisionAvoidanceLidarField::Parameter CollisionAvoidanceLidarField::get_parameter(
  const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  ros_node.declare_parameter<double>(
    "velocity.warnfield.linear", default_parameter.velocity_warnfield_linear);
    ros_node.declare_parameter<double>(
      "velocity.warnfield.angular", default_parameter.velocity_warnfield_angular);
  Parameter parameter;

  parameter.velocity_warnfield_linear = ros_node.get_parameter("velocity.warnfield.linear").as_double();
  parameter.velocity_warnfield_angular = ros_node.get_parameter("velocity.warnfield.angular").as_double();

  return parameter;
}

CollisionAvoidanceLidarField::CollisionAvoidanceLidarField()
  : rclcpp::Node("collision_avoidance_lidar_field")
  , _parameter(get_parameter({}, *this))
{
  _sub_twist = create_subscription<geometry_msgs::msg::Twist>(
    "in/twist", 
    rclcpp::QoS(2).best_effort(), 
    std::bind(&CollisionAvoidanceLidarField::callbackTwist, this, std::placeholders::_1)
  );
  _sub_field = create_subscription<edu_perception::msg::LidarFieldEvaluation>(
    "in/field", 
    rclcpp::QoS(2).reliable().transient_local(), 
    std::bind(&CollisionAvoidanceLidarField::callbackLidarField, this, std::placeholders::_1)
  );
  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "out/twist", rclcpp::QoS(2).reliable()
  );
}

CollisionAvoidanceLidarField::~CollisionAvoidanceLidarField()
{

}

void CollisionAvoidanceLidarField::callbackTwist(std::shared_ptr<const geometry_msgs::msg::Twist> msg)
{
  geometry_msgs::msg::Twist twist_out(*msg);

  if (_safetyfield_active) {
    twist_out.linear.x = 0.0;
    twist_out.linear.y = 0.0;
    twist_out.linear.z = 0.0;

    twist_out.angular.x = 0.0;
    twist_out.angular.y = 0.0;
    twist_out.angular.z = 0.0;
  }
  else if (_warnfield_active) {
    limit_velocity(twist_out.linear.x, _parameter.velocity_warnfield_linear);
    limit_velocity(twist_out.linear.y, _parameter.velocity_warnfield_linear);
    limit_velocity(twist_out.linear.z, _parameter.velocity_warnfield_linear);

    limit_velocity(twist_out.angular.x, _parameter.velocity_warnfield_angular);
    limit_velocity(twist_out.angular.y, _parameter.velocity_warnfield_angular);
    limit_velocity(twist_out.angular.z, _parameter.velocity_warnfield_angular);
  }

  _pub_twist->publish(twist_out);
}

void CollisionAvoidanceLidarField::callbackLidarField(
  std::shared_ptr<const edu_perception::msg::LidarFieldEvaluation> msg)
{
  // reset old state
  _warnfield_active = false;
  _safetyfield_active = false;

  for (const auto& field : msg->fields) {
    if (field.name == "warnfeld" && field.state == edu_perception::msg::LidarField::INFRINGED) {
      _warnfield_active = true;
    }
    else if (field.name == "schutzfeld" && field.state == edu_perception::msg::LidarField::INFRINGED) {
      _safetyfield_active = true;
    }
  }
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::CollisionAvoidanceLidarField>());
  rclcpp::shutdown();

  return 0;
}