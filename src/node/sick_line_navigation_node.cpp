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

static inline void set_lighting_stop(rclcpp::Publisher<edu_robot::msg::SetLightingColor>& publisher) {
  set_lighting(publisher, "all", 0, 100, 0, edu_robot::msg::SetLightingColor::ROTATION);
}

static std::string get_mode_string(const edu_robot::msg::Mode mode)
{
  std::string mode_string;

  if (mode.mode & edu_robot::msg::Mode::INACTIVE) {
    mode_string += "INACTIVE|";
  }
  if (mode.mode & edu_robot::msg::Mode::REMOTE_CONTROLLED) {
    mode_string += "REMOTE CONTROLLED|";
  }
  if (mode.mode & edu_robot::msg::Mode::AUTONOMOUS) {
    mode_string += "FLEET|";
  }
  if (mode.feature_mode & edu_robot::msg::Mode::COLLISION_AVOIDANCE) {
    mode_string += "COLLISION_AVOIDANCE|";
  }
  if (mode.feature_mode & edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE) {
    mode_string += "COLLISION_AVOIDANCE_OVERRIDE|";
  }  
  if (mode.drive_kinematic & edu_robot::msg::Mode::SKID_DRIVE) {
    mode_string += "SKID_DRIVE|";
  }
  if (mode.drive_kinematic & edu_robot::msg::Mode::MECANUM_DRIVE) {
    mode_string += "MECANUM_DRIVE|";
  }

  if (mode_string.empty() == false) {
    mode_string.pop_back();
  }

  return mode_string;
}

static void disable(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  using ResponseFuture = rclcpp::Client<edu_robot::srv::SetMode>::SharedFutureWithRequest;

  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.mode = edu_robot::msg::Mode::INACTIVE;

  RCLCPP_INFO(node.get_logger(), "Send set mode request mode = INACTIVE.");
  service_client.async_send_request(
    request,
    [logger = node.get_logger()](ResponseFuture future) {
      const auto response = future.get();

      if ((response.second->state.mode.mode & response.first->mode.mode) == false) {
        RCLCPP_ERROR_STREAM(logger, "Can't disable robot! Robot is in mode = " << get_mode_string(response.second->state.mode));
        return;
      }

      RCLCPP_INFO(logger, "Set mode INACTIVE successfully.");
      RCLCPP_INFO(logger, "Current mode of the robot is = %s", get_mode_string(response.second->state.mode).c_str());      
    }
  );
}


SickLineNavigation::Parameter SickLineNavigation::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<double>("move_velocity.slow", default_parameter.move_velocity_slow);
  ros_node.declare_parameter<double>("move_velocity.middle", default_parameter.move_velocity_middle);
  ros_node.declare_parameter<double>("move_velocity.fast", default_parameter.move_velocity_fast);
  ros_node.declare_parameter<double>("stop_time", default_parameter.stop_time);

  parameter.move_velocity_slow = ros_node.get_parameter("move_velocity.slow").as_double();
  parameter.move_velocity_middle = ros_node.get_parameter("move_velocity.middle").as_double();
  parameter.move_velocity_fast = ros_node.get_parameter("move_velocity.fast").as_double();
  parameter.stop_time = ros_node.get_parameter("stop_time").as_double();

  return parameter;
}

SickLineNavigation::SickLineNavigation()
  : rclcpp::Node("sick_line_navigation")
  , _parameter(get_parameter({}, *this))
{
  _processing_data.requested_velocity = _parameter.move_velocity_slow;

  // ROS Related
  // Topics
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
  _sub_field_evaluation = create_subscription<edu_perception::msg::LidarFieldEvaluation>(
    "in/field_evaluation", 
    rclcpp::QoS(1).reliable().transient_local(), 
    std::bind(&SickLineNavigation::callbackFieldEvaluation, this, std::placeholders::_1)
  );

  // Services
  _client_set_mode = create_client<edu_robot::srv::SetMode>("set_mode");

  // Starting Timer --> Starting Processing
  _timer_processing = create_timer(100ms, std::bind(&SickLineNavigation::process, this));
}

void SickLineNavigation::callbackOnTrack(std::shared_ptr<const std_msgs::msg::Bool> msg)
{
  // RCLCPP_INFO(get_logger(), "received on track flag: %i.", msg->data);
  _processing_data.on_track = msg->data;
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
  // 10. disable robot
  // 11. schnell
  // 12. mittel schnell
  // 13. langsam
  // 20. stop for given time
  
  switch (msg->code) {
    // Lighting
    case 1: set_lighting_turn_left (*_pub_lighting_color); break;
    case 2: set_lighting_turn_right(*_pub_lighting_color); break;
    case 3: set_lighting_default   (*_pub_lighting_color); break;
    case 4: set_lighting_police    (*_pub_lighting_color); break;
    case 5: set_lighting_warning   (*_pub_lighting_color); break;

    // Moving Velocity
    case 10: disable(*this, *_client_set_mode); break;
    case 11: _processing_data.requested_velocity = _parameter.move_velocity_slow; break;
    case 12: _processing_data.requested_velocity = _parameter.move_velocity_middle; break;
    case 13: _processing_data.requested_velocity = _parameter.move_velocity_fast; break;

    // Special Actions
    // Stop/Halt for given time
    case 20: 
      _processing_data.stop_active = true;
      set_lighting_stop(*_pub_lighting_color);
      _timer_process_stopping = create_timer(
        std::chrono::round<std::chrono::milliseconds>(std::chrono::duration<float>(_parameter.stop_time)),
      std::bind(&SickLineNavigation::deactivateStop, this)
      );
      break;

    // Not supported code
    default:
      RCLCPP_ERROR(get_logger(), "un suported code %i.", msg->code);
      break;
  }
}

void SickLineNavigation::callbackFieldEvaluation(std::shared_ptr<const edu_perception::msg::LidarFieldEvaluation> msg)
{
  for (const auto& field : msg->fields) {
    if (field.name == "warnfeld") {
      _processing_data.warnfeld_active = field.state == edu_perception::msg::LidarField::INFRINGED; 
    }
    else if (field.name == "schutzfeld") {
      _processing_data.schutzfeld_active = field.state == edu_perception::msg::LidarField::INFRINGED;
    }
  }
}

void SickLineNavigation::deactivateStop()
{
  _processing_data.stop_active = false;
  // cancel timer so we get single shot behaviour
  _timer_process_stopping->cancel();
}

void SickLineNavigation::process()
{
  geometry_msgs::msg::Twist twist;

  if (_processing_data.on_track == false) {
    twist.linear.x = 0.0;
  }
  // robot is on track
  else if (_processing_data.stop_active) {
    // stop is active --> send velocity 0 until stop ended
    twist.linear.x = 0.0;
  }
  else if (_processing_data.schutzfeld_active) {
    // some object is in Schutzfeld --> stop
    twist.linear.x = 0.0;
  }
  // no collision with an object
  else if (_processing_data.warnfeld_active) {
    // some object is in Warnfeld --> slow down
    twist.linear.x = _parameter.move_velocity_slow;
  }
  // no close object around robot
  else {
    twist.linear.x = _processing_data.requested_velocity;
  }
  
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