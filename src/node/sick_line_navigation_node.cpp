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

static void enable(rclcpp::Node& node, rclcpp::Client<edu_robot::srv::SetMode>& service_client)
{
  using ResponseFuture = rclcpp::Client<edu_robot::srv::SetMode>::SharedFutureWithRequest;

  auto request = std::make_shared<edu_robot::srv::SetMode::Request>();
  request->mode.mode = edu_robot::msg::Mode::AUTONOMOUS;

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

static void send_drive_action(
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>& publisher, const std::string& action)
{
  std_msgs::msg::String msg;
  msg.data = action;
  publisher->publish(msg);
}

static void send_lifecycle_node_transition(
  const std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client,
  const rclcpp::Logger& logger,
  lifecycle_msgs::msg::Transition::_id_type transition)
{
  const std::string service_name = client->get_service_name();
  const std::string node_name = service_name.substr(0, service_name.find("/change_state"));

  // guarantee that service is available
  if (client->wait_for_service(5s) == false) {
    RCLCPP_ERROR(logger, "service %s not available.", client->get_service_name());
  }

  // request transition for lifecycle node
  auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  change_state_request->transition.id = transition;

  client->async_send_request(
    change_state_request,
    [node_name, logger](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
      auto response = future.get();

      if (response->success) {
        RCLCPP_INFO(logger, "successfully changed state of node %s.", node_name.c_str());
      } else {
        RCLCPP_ERROR(logger, "failed to change state of node %s.", node_name.c_str());
      }
    }
  );
}

void SickLineNavigation::performFullTurn()
{
  if (_action_client_rotate->wait_for_action_server(5s) == false) {
    RCLCPP_ERROR(get_logger(), "action server not available after waiting");
    return;
  }

  // disable driving while turning
  send_lifecycle_node_transition(
    _client_change_state, get_logger(), lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE
  );
  // stop robot before turning
  std::lock_guard<std::mutex> lock(_processing_data.mutex);
  const auto drive_velocity = _processing_data.requested_velocity;
  _processing_data.requested_velocity = 0.0;

  // activate action by sending goal
  auto goal_msg = edu_fleet::action::RobotRotate::Goal();
  goal_msg.relative_yaw = M_PI; // 180 degree
  goal_msg.yaw_rate = _parameter.turning_velocity;

  auto send_goal_options = rclcpp_action::Client<edu_fleet::action::RobotRotate>::SendGoalOptions();
  send_goal_options.result_callback = [this, drive_velocity](
    const rclcpp_action::ClientGoalHandle<edu_fleet::action::RobotRotate>::WrappedResult & result) 
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "action succeeded");
          RCLCPP_INFO(get_logger(), "remaining yaw error: %f", result.result->error_yaw);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "action was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(get_logger(), "action was canceled");
          break;
        default:
          RCLCPP_ERROR(get_logger(), "unknown result code");
          break;
      }

      // after turn enable driving again
      send_lifecycle_node_transition(
        _client_change_state, get_logger(), lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE
      );
      // restore previous velocity
      std::lock_guard<std::mutex> lock(_processing_data.mutex);
      _processing_data.requested_velocity = drive_velocity;
    };

  _action_client_rotate->async_send_goal(goal_msg, send_goal_options);
}

SickLineNavigation::Parameter SickLineNavigation::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<double>("move_velocity.slow", default_parameter.move_velocity_slow);
  ros_node.declare_parameter<double>("move_velocity.middle", default_parameter.move_velocity_middle);
  ros_node.declare_parameter<double>("move_velocity.fast", default_parameter.move_velocity_fast);
  ros_node.declare_parameter<double>("stop_time", default_parameter.stop_time);
  ros_node.declare_parameter<std::string>(
    "line_controller_node_name", default_parameter.line_controller_node_name);

  parameter.move_velocity_slow = ros_node.get_parameter("move_velocity.slow").as_double();
  parameter.move_velocity_middle = ros_node.get_parameter("move_velocity.middle").as_double();
  parameter.move_velocity_fast = ros_node.get_parameter("move_velocity.fast").as_double();
  parameter.stop_time = ros_node.get_parameter("stop_time").as_double();
  parameter.line_controller_node_name = ros_node.get_parameter("line_controller_node_name").as_string();

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
  _pub_drive_action = create_publisher<std_msgs::msg::String>(
    "out/drive_action",
    rclcpp::QoS(2).reliable()
  );
  _sub_on_track = create_subscription<std_msgs::msg::Bool>(
    "in/on_track", 
    rclcpp::QoS(2).transient_local(), 
    std::bind(&SickLineNavigation::callbackOnTrack, this, std::placeholders::_1)
  );
  _sub_code = create_subscription<sick_lidar_localization_msgs::msg::CodeMeasurementMessage0304>(
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
  _client_change_state = create_client<lifecycle_msgs::srv::ChangeState>(
    _parameter.line_controller_node_name + "/change_state"
  );

  // Activate line controller node
  send_lifecycle_node_transition(
    _client_change_state, get_logger(), lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE
  );
  send_lifecycle_node_transition(
    _client_change_state, get_logger(), lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE
  );

  // Actions
  _action_client_rotate = rclcpp_action::create_client<edu_fleet::action::RobotRotate>(this, "robot_rotate");

  // Starting Timer --> Starting Processing
  _timer_processing = create_timer(100ms, std::bind(&SickLineNavigation::process, this));
}

SickLineNavigation::~SickLineNavigation()
{
  // Deactivate line controller node
  send_lifecycle_node_transition(
    _client_change_state, get_logger(), lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE
  );
  send_lifecycle_node_transition(
    _client_change_state, get_logger(), lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP
  );
}

void SickLineNavigation::callbackOnTrack(std::shared_ptr<const std_msgs::msg::Bool> msg)
{
  // RCLCPP_INFO(get_logger(), "received on track flag: %i.", msg->data);
  _processing_data.on_track = msg->data;
}

void SickLineNavigation::callbackCode(std::shared_ptr<const sick_lidar_localization_msgs::msg::CodeMeasurementMessage0304> msg)
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
  // 14. vorwärts fahren
  // 15. rückwärts fahren
  // 30. drive straight at next switch
  // 31. drive left at next switch
  // 32. drive right at next switch
  // 33. 180 degree turn
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
    // Velocity Slow
    case 11: { 
        std::lock_guard<std::mutex> lock(_processing_data.mutex);
        _processing_data.requested_velocity = _parameter.move_velocity_slow;
      }
      break;
    // Velocity Middle
    case 12: {
        std::lock_guard<std::mutex> lock(_processing_data.mutex);
        _processing_data.requested_velocity = _parameter.move_velocity_middle;
      }
      break;
    // Velocity Fast
    case 13: {
        std::lock_guard<std::mutex> lock(_processing_data.mutex);
        _processing_data.requested_velocity = _parameter.move_velocity_fast;
      }
      break;
    case 14: _processing_data.drive_backwards = false; break;
    case 15: _processing_data.drive_backwards = true; break;

    // Turning
    case 30: send_drive_action(_pub_drive_action, "straight"); break;
    case 31: send_drive_action(_pub_drive_action, "turn_left"); break;
    case 32: send_drive_action(_pub_drive_action, "turn_right"); break;
    case 33: performFullTurn(); break;

    // Special Actions
    // Stop/Halt for given time
    case 20: 
      _processing_data.stop_active = true;
      disable(*this, *_client_set_mode);
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
  enable(*this, *_client_set_mode);
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
    std::lock_guard<std::mutex> lock(_processing_data.mutex);
    twist.linear.x = _parameter.move_velocity_slow;
  }
  // no close object around robot
  else {
    std::lock_guard<std::mutex> lock(_processing_data.mutex);
    twist.linear.x = _processing_data.requested_velocity;
  }

  // handle backwards driving
  if (_processing_data.drive_backwards) {
    twist.linear.x = -twist.linear.x;
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