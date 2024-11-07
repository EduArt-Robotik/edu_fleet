#include "sick_line_navigation_node.hpp"

namespace eduart {
namespace fleet {

using namespace std::chrono_literals;

// helper for set lighting
static void set_lighting(
  rclcpp::Publisher<edu_robot::msg::SetLightingColor>& publisher, const std::string& lighting_name,
  const std::uint8_t r, const std::uint8_t g, const std::uint8_t b, const std::uint8_t mode)
{
  edu_robot::msg::SetLightingColor msg;

  msg.r = r;
  msg.g = g;
  msg.b = b;

  msg.brightness.data = 0.7;
  msg.lighting_name = lighting_name;
  msg.mode = mode;

  publisher.publish(msg);
}  

static inline void set_lighting_default(rclcpp::Publisher<edu_robot::msg::SetLightingColor>& publisher) {
  set_lighting(publisher, "all", 34, 34, 34, edu_robot::msg::SetLightingColor::FLASH);
}

static inline void set_lighting_turn_left(rclcpp::Publisher<edu_robot::msg::SetLightingColor>& publisher) {
  set_lighting(publisher, "left_side", 14, 11, 0, edu_robot::msg::SetLightingColor::FLASH);
}

static inline void set_lighting_turn_right(rclcpp::Publisher<edu_robot::msg::SetLightingColor>& publisher) {
  set_lighting(publisher, "right_side", 14, 11, 0, edu_robot::msg::SetLightingColor::FLASH);
}

static inline void set_lighting_warning(rclcpp::Publisher<edu_robot::msg::SetLightingColor>& publisher) {
  set_lighting(publisher, "all", 14, 11, 0, edu_robot::msg::SetLightingColor::FLASH);
}

static inline void set_lighting_police(rclcpp::Publisher<edu_robot::msg::SetLightingColor>& publisher) {
  set_lighting(publisher, "all", 0, 0, 255, edu_robot::msg::SetLightingColor::ROTATION);
}


SickLineNavigation::Parameter SickLineNavigation::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<double>("move_velocity.slow", default_parameter.move_velocity_slow);
  ros_node.declare_parameter<double>("move_velocity.fast", default_parameter.move_velocity_fast);

  parameter.move_velocity_slow = ros_node.get_parameter("move_velocity.slow").as_double();
  parameter.move_velocity_fast = ros_node.get_parameter("move_velocity.fast").as_double();

  return default_parameter;
}

SickLineNavigation::SickLineNavigation()
  : rclcpp::Node("sick_line_navigation")
  , _parameter(get_parameter({}, *this))
  , _on_track(false)
  , _current_velocity(_parameter.move_velocity_slow)
{
  // ROS Related
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/cmd_vel",
    rclcpp::QoS(10).reliable()
  );
  _pub_lighting_color = create_publisher<edu_robot::msg::SetLightingColor>(
    "out/set_lighting_color", 
    rclcpp::QoS(2).reliable()
  );
  _sub_on_track = create_subscription<std_msgs::msg::Bool>(
    "in/on_track", 
    rclcpp::QoS(2).transient_local(), 
    std::bind(&SickLineNavigation::callbackOnTrack, this, std::placeholders::_1)
  );
  _sub_code = create_subscription<sick_lidar_localization::msg::CodeMeasurementMessage0304>(
    "in/code", 
    rclcpp::QoS(10).reliable(), 
    std::bind(&SickLineNavigation::callbackCode, this, std::placeholders::_1)
  );
  _timer_processing = create_timer(100ms, std::bind(&SickLineNavigation::process, this));
}

void SickLineNavigation::callbackOnTrack(std::shared_ptr<const std_msgs::msg::Bool> msg)
{
  RCLCPP_INFO(get_logger(), "received on track flag: %i.", msg->data);
  _on_track = msg->data;
}

void SickLineNavigation::callbackCode(std::shared_ptr<const sick_lidar_localization::msg::CodeMeasurementMessage0304> msg)
{
  RCLCPP_INFO(get_logger(), "received code \"%i\".", msg->code);
  // Following codes were defined:
  // 1. links blinken
  // 2. rechts blinken
  // 3. default licht
  // 4. blau rotierend (police)
  // 5. warnblinklicht
  // 6. schnell
  // 7. langsam
  
  switch (msg->code) {
    // Lighting
    case 1: set_lighting_turn_left (*_pub_lighting_color); break;
    case 2: set_lighting_turn_right(*_pub_lighting_color); break;
    case 3: set_lighting_default   (*_pub_lighting_color); break;
    case 4: set_lighting_police    (*_pub_lighting_color); break;
    case 5: set_lighting_warning   (*_pub_lighting_color); break;

    // Moving Velocity
    case 6: _current_velocity = _parameter.move_velocity_fast; break;
    case 7: _current_velocity = _parameter.move_velocity_slow; break;
    
    // Not supported code
    default:
      RCLCPP_ERROR(get_logger(), "un suported code %i.", msg->code);
      break;
  }
}

void SickLineNavigation::process()
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x = _on_track ? _current_velocity : 0.0;
  
  _pub_velocity->publish(twist);
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::SickLineNavigation>());
  rclcpp::shutdown();

  return 0;
}