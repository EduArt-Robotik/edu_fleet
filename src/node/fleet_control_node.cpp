#include "fleet_control_node.hpp"

namespace eduart {
namespace fleet {

ControlNode::Parameter ControlNode::get_parameter(rclcpp::Node& ros_node)
{
  ControlNode::Parameter parameter;

  return parameter;
}

ControlNode::ControlNode()
  : rclcpp::Node("fleet_control_node")
  , _parameter(get_parameter(*this))
{

}

ControlNode::~ControlNode()
{

}

void ControlNode::callbackTwistFleet(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg)
{

}

} // end namespace fleet
} // end namespace eduart
