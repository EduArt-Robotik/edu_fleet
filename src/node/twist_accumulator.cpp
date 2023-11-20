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
  ros_node.declare_parameter<float>("timeout", parameter.timeout);
  
  parameter.num_subscriptions = ros_node.get_parameter("num_subscription").as_int();
  parameter.timeout = ros_node.get_parameter("timeout").as_double();

  return parameter;
}

TwistAccumulator::TwistAccumulator()
  : rclcpp::Node("twist_accumulator")
  , _parameter(get_parameter(*this))
{
  const auto now = get_clock()->now();

  for (std::size_t i = 0; i < _parameter.num_subscriptions; ++i) {
    _sub_twist.emplace_back(create_subscription<geometry_msgs::msg::Twist>(
      std::string("twist/input_") + std::to_string(i),
      rclcpp::QoS(1).best_effort(),
      // std::bind(&TwistAccumulator::callbackTwistInput, this, std::placeholders::_1, i)
      [this, i](std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg) {
        callbackTwistInput(twist_msg, i);
      }
    ));
    _current_input.emplace_back(create_null_twist());
    _stamp_last_input_update.emplace_back(now);
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

static void clear_outdated_twist(
  std::vector<geometry_msgs::msg::Twist>& current_input, const std::vector<rclcpp::Time>& stamp_last_input_update,
  const rclcpp::Time& stamp_now, const float timeout)
{
  for (std::size_t channel = 0; channel < current_input.size(); ++channel) {
    if ((stamp_now - stamp_last_input_update[channel]).seconds() >= timeout) {
      current_input[channel] = create_null_twist();
    }
  }
}

void TwistAccumulator::callbackTwistInput(
  std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg, const std::size_t index)
{
  const auto now = get_clock()->now();
  _current_input[index] = *twist_msg;
  _stamp_last_input_update[index] = now;

  clear_outdated_twist(
    _current_input, _stamp_last_input_update, now, 0.2f
  );
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