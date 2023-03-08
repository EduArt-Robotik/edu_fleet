#include "twist_accumulator.hpp"
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <string>
#include <rclcpp/executors.hpp>

namespace eduart {
namespace fleet {

static geometry_msgs::msg::Twist create_null_twist()
{
  geometry_msgs::msg::Twist null;

  null.linear.x = 0.0;
  null.linear.y = 0.0;
  null.linear.z = 0.0;

  null.angular.x = 0.0;
  null.angular.y = 0.0;
  null.angular.z = 0.0;

  return null;
}

TwistAccumulator::Parameter TwistAccumulator::get_parameter(rclcpp::Node &ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<int>("num_subscription", parameter.num_subscriptions);
  
  parameter.num_subscriptions = ros_node.get_parameter("num_subscription").as_int();

  return parameter;
}

TwistAccumulator::TwistAccumulator()
  : rclcpp::Node("twist_accumulator")
{
  for (std::size_t i = 0; i < _parameter.num_subscriptions; ++i) {
    _sub_twist.emplace_back(create_subscription<geometry_msgs::msg::Twist>(
      std::string("twist/input_") + std::to_string(i),
      rclcpp::QoS(1).reliable(),
      // std::bind(&TwistAccumulator::callbackTwistInput, this, std::placeholders::_1, i)
      [this, i](std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg) {
        callbackTwistInput(twist_msg, i);
      }
    ));
    _current_input.emplace_back(create_null_twist());
  }

  _pub_twist = create_publisher<geometry_msgs::msg::Twist>(
    "twist/output", rclcpp::QoS(1).reliable()
  );
}

TwistAccumulator::~TwistAccumulator()
{

}

static geometry_msgs::msg::Twist accumulate_twist(const std::vector<geometry_msgs::msg::Twist>& input)
{
  auto output = create_null_twist();

  for (const auto& twist_input : input) {
    output.linear.x += twist_input.linear.x;
    output.linear.y += twist_input.linear.y;
    output.linear.z += twist_input.linear.z;

    output.angular.x += twist_input.angular.x;
    output.angular.y += twist_input.angular.y;
    output.angular.z += twist_input.angular.z;      
  }

  return output;
}

void TwistAccumulator::callbackTwistInput(
  std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg, const std::size_t index)
{
  _current_input[index] = *twist_msg;
  const auto twist_out = accumulate_twist(_current_input);
  _pub_twist->publish(twist_out);
}  

} // end namespace fleet
} // end namespace eduart


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<eduart::fleet::TwistAccumulator>());
  rclcpp::shutdown();

  return 0;
}