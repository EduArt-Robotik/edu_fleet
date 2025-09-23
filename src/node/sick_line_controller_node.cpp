#include "sick_line_controller_node.hpp"

#include <rclcpp/executors.hpp>

namespace eduart {
namespace fleet {

// Helper Functions
static inline bool is_left(const std::uint8_t cnt_lpc) {
  return cnt_lpc & (1 << 0);
}
static inline bool is_middle(const std::uint8_t cnt_lpc) {
  return cnt_lpc & (1 << 1);
}
static inline bool is_right(const std::uint8_t cnt_lpc) {
  return cnt_lpc & (1 << 2);
}

static std::optional<std::size_t> get_index_from_source_id(
  const std::vector<std::int64_t>& source_ids, const std::int32_t source_id)
{
  for (std::size_t i = 0; i < source_ids.size(); ++i) {
    // First get correct index i of received line measurement.
    if (source_id == source_ids[i]) {
      return i;
    }
  }

  return std::nullopt;
}

SickLineController::Parameter SickLineController::get_parameter(
  const Parameter &default_parameter, rclcpp_lifecycle::LifecycleNode &ros_node)
{
  ros_node.declare_parameter<double>("d_x", default_parameter.d_x);
  ros_node.declare_parameter<double>("max_error_on_track", default_parameter.max_error_on_track);
  ros_node.declare_parameter<double>("max_error_yaw", default_parameter.max_error_yaw);
  ros_node.declare_parameter<std::vector<std::int64_t>>("source_ids", default_parameter.source_ids);
  ros_node.declare_parameter<std::vector<std::int64_t>>("front_source_ids", default_parameter.front_source_ids);
  ros_node.declare_parameter<std::vector<std::int64_t>>("rear_source_ids", default_parameter.rear_source_ids);

  ros_node.declare_parameter<double>("pid.linear.kp", default_parameter.pid.stay_on_line.kp);
  ros_node.declare_parameter<double>("pid.linear.limit", default_parameter.pid.stay_on_line.limit);
  ros_node.declare_parameter<double>("pid.angular.kp", default_parameter.pid.orientate_to_line.kp);
  ros_node.declare_parameter<double>("pid.angular.limit", default_parameter.pid.orientate_to_line.limit);

  Parameter parameter;

  parameter.d_x = ros_node.get_parameter("d_x").as_double();
  parameter.max_error_on_track = ros_node.get_parameter("max_error_on_track").as_double();
  parameter.max_error_yaw = ros_node.get_parameter("max_error_yaw").as_double();
  parameter.source_ids = ros_node.get_parameter("source_ids").as_integer_array();
  parameter.front_source_ids = ros_node.get_parameter("front_source_ids").as_integer_array();
  parameter.rear_source_ids = ros_node.get_parameter("rear_source_ids").as_integer_array();

  parameter.pid.stay_on_line.kp = ros_node.get_parameter("pid.linear.kp").as_double();
  parameter.pid.stay_on_line.limit = ros_node.get_parameter("pid.linear.limit").as_double();
  parameter.pid.orientate_to_line.kp = ros_node.get_parameter("pid.angular.kp").as_double();
  parameter.pid.orientate_to_line.limit = ros_node.get_parameter("pid.angular.limit").as_double();

  // valid source ids
  if (parameter.source_ids.size() != NUM_SENSORS) {
    RCLCPP_FATAL(rclcpp::get_logger("line_controller"), "number of given source ids does not match NUM_SENSORS.");
    throw std::invalid_argument("number of given source ids does not match NUM_SENSORS.");
  }

  return parameter;
}

SickLineController::SickLineController()
  : rclcpp_lifecycle::LifecycleNode("line_controller")
  , _parameter(get_parameter({}, *this))
{

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SickLineController::on_configure(
  const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "configuring node.");

  // Configuring ROS Topics and Services
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/velocity", rclcpp::QoS(2).reliable()
  );
  _pub_on_track = create_publisher<std_msgs::msg::Bool>(
    "out/on_track", 
    rclcpp::QoS(2).transient_local()
  );
  _sub_line_sensor = create_subscription<sick_lidar_localization_msgs::msg::LineMeasurementMessage0404>(
    "in/line_detection",
    rclcpp::QoS(10).reliable(),
    std::bind(&SickLineController::callbackLineSensor, this, std::placeholders::_1)
  );
  _sub_action = create_subscription<std_msgs::msg::String>(
    "in/drive_action", 
    rclcpp::QoS(2).reliable(),
    std::bind(&SickLineController::callbackAction, this, std::placeholders::_1)
  );

  // Initializing Processing Data
  _processing_data.line_distance_received.fill(false);
  _processing_data.line_distance.fill(0.0);
  _processing_data.valid_line_distance.fill(false);
  _processing_data.cnt_lpc.fill(0);
  _processing_data.current_telegram = 0;

  _processing_data.stay_on_line = std::make_shared<controller::Pid>(_parameter.pid.stay_on_line);
  _processing_data.orientate_to_line = std::make_shared<controller::Pid>(_parameter.pid.orientate_to_line);
  _processing_data.stay_on_line->reset();
  _processing_data.orientate_to_line->reset();
  _processing_data.stamp_last_processing = get_clock()->now();

  _processing_data.action.fill(Action::STRAIGHT);
  _processing_data.track.fill(Track::MIDDLE);

  // Ready for Processing
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SickLineController::on_cleanup(
  const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "cleaning up node.");

  // Destroying ROS Publishers and Services
  _pub_velocity.reset();
  _pub_on_track.reset();
  _sub_line_sensor.reset();
  _sub_action.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SickLineController::on_shutdown(
  const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "shutting down node.");

  // Destroying ROS Publishers and Services
  _pub_velocity.reset();
  _pub_on_track.reset();
  _sub_line_sensor.reset();
  _sub_action.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SickLineController::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_logger(), "activating node.");
  (void)previous_state;
  return rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SickLineController::on_deactivate(
  const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(get_logger(), "deactivating node.");
  (void)previous_state;
  return rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
}

void SickLineController::callbackLineSensor(const sick_lidar_localization_msgs::msg::LineMeasurementMessage0404& msg)
{
  // Start new measurement cycle if telegram number changed.
  if (msg.telegram_count != _processing_data.current_telegram) {
    // RCLCPP_INFO(get_logger(), "start new line measurement set.");
    _processing_data.current_telegram = msg.telegram_count;
    _processing_data.line_distance_received.fill(false);
    _processing_data.line_distance.fill(0.0);
    _processing_data.valid_line_distance.fill(false);
    _processing_data.cnt_lpc.fill(0);
  }

  // Assign line measurement.
  const auto index = get_index_from_source_id(_parameter.source_ids, msg.source_id);
  // RCLCPP_INFO(get_logger(), "process source id %u.", msg.source_id);

  if (index.has_value() == false) {
    // not source id found
    // \todo maybe add a warning here
    return;
  }

  // Found correct index i.
  _processing_data.line_distance_received[*index] = true;
  _processing_data.valid_line_distance[*index] = false; // default value
  _processing_data.cnt_lpc[*index] = msg.cnt_lpc;

  switch (_processing_data.action[*index]) {
    case Action::STRAIGHT:
      // want to drive straight 
      if (is_middle(msg.cnt_lpc)) {
        _processing_data.valid_line_distance[*index] = true;
        _processing_data.line_distance[*index] = -msg.lcp2 / 1000.0f; // convert into meter
        _processing_data.track[*index] = Track::MIDDLE;
        // RCLCPP_INFO(get_logger(), "drive straight");
      }
      break;

    case Action::TURN_LEFT:
      if (is_left(msg.cnt_lpc) == false && is_middle(msg.cnt_lpc) == true) {
        if (_processing_data.track[*index] == Track::MIDDLE) {
          // still on middle --> turning not started yet
          _processing_data.track[*index] = Track::MIDDLE;
          // RCLCPP_INFO(get_logger(), "still on middle --> turning left not started yet");
        }
        else if (_processing_data.track[*index] == Track::LEFT) {
          // turing left is finished --> going back to middle track
          _processing_data.track[*index] = Track::MIDDLE;
          _processing_data.action[*index] = Action::STRAIGHT;
          RCLCPP_INFO(get_logger(), "turing left is finished --> going back to middle track");
        }
        else {
          // \todo think about this case!
          RCLCPP_WARN(get_logger(), "unexpected track");
        }

        _processing_data.valid_line_distance[*index] = true;
        _processing_data.line_distance[*index] = -msg.lcp2 / 1000.0f; // convert into meter
      }
      else if (is_left(msg.cnt_lpc)) {
        // left track detected --> turning left
          RCLCPP_INFO(get_logger(), "left track detected --> turning left");
        _processing_data.valid_line_distance[*index] = true;
        _processing_data.line_distance[*index] = -msg.lcp3 / 1000.0f; // convert into meter
        _processing_data.track[*index] = Track::LEFT;        
      }
    break;

    case Action::TURN_RIGHT:
      if (is_right(msg.cnt_lpc) == false && is_middle(msg.cnt_lpc) == true) {
        if (_processing_data.track[*index] == Track::MIDDLE) {
          // still on middle --> turning not started yet
          // RCLCPP_INFO(get_logger(), "still on middle --> turning right not started yet");
          _processing_data.track[*index] = Track::MIDDLE;
        }
        else if (_processing_data.track[*index] == Track::RIGHT) {
          // turing right is finished --> going back to middle track
          RCLCPP_INFO(get_logger(), "turing right is finished --> going back to middle track");
          _processing_data.track[*index] = Track::MIDDLE;
          _processing_data.action[*index] = Action::STRAIGHT;
        }
        else {
          // \todo think about this case!
          RCLCPP_WARN(get_logger(), "unexpected track");
        }
          
        _processing_data.valid_line_distance[*index] = true;
        _processing_data.line_distance[*index] = -msg.lcp2 / 1000.0f; // convert into meter
      }
      else if (is_right(msg.cnt_lpc)) {
        // right track detected --> turning right
          RCLCPP_INFO(get_logger(), "right track detected --> turning right");
        _processing_data.valid_line_distance[*index] = true;
        _processing_data.line_distance[*index] = -msg.lcp1 / 1000.0f; // convert into meter
        _processing_data.track[*index] = Track::RIGHT;        
      }
    break;

    default:
      RCLCPP_WARN(get_logger(), "not handled action");
    break;
  }

  // RCLCPP_INFO(get_logger(), "source id %u, action %u, track %u.", msg.source_id,
    // static_cast<unsigned int>(_processing_data.action[*index]), static_cast<unsigned int>(_processing_data.track[*index]));


  // Check if all needed measurements are received.
  for (const auto received : _processing_data.line_distance_received) {
    if (received == false) {
      // Minium one measurement is missing --> return
      return;
    }
  }
  //else: Line measurements are finished.
  processDistances();
}

void SickLineController::callbackAction(const std_msgs::msg::String& msg)
{
  // \todo clarify if a active action should be canceled?
  if (msg.data == "turn_left") {
    _processing_data.action.fill(Action::TURN_LEFT);
    RCLCPP_INFO(get_logger(), "turn left command received.");
  }
  else if (msg.data == "turn_right") {
    _processing_data.action.fill(Action::TURN_RIGHT);
    RCLCPP_INFO(get_logger(), "turn right command received.");
  }
  else if (msg.data == "straight") {
    _processing_data.action.fill(Action::STRAIGHT);
    RCLCPP_INFO(get_logger(), "drive straight command received.");
  }
}

void SickLineController::processDistances()
{
  // Check if all measurements are valid.
  for (const auto valid : _processing_data.valid_line_distance) {
    if (valid == false) {
      RCLCPP_ERROR(get_logger(), "Minium one measurement is not valid --> stop robot");

      // Publish command to stop the robot.
      _pub_on_track->publish(std_msgs::msg::Bool());
      _pub_velocity->publish(geometry_msgs::msg::Twist());
      return;
    }
  }
  //else: valid measurement.
  // RCLCPP_INFO(get_logger(), "line measurements complete.");
  // Get dt
  const auto stamp_now = get_clock()->now();
  const double dt = std::min(0.1, (stamp_now - _processing_data.stamp_last_processing).seconds());

  // Calculate Yaw error.
  const std::size_t index_front = getBestIndex(_parameter.front_source_ids);
  const std::size_t index_rear  = getBestIndex(_parameter.rear_source_ids);
  const double error_line = _processing_data.line_distance[index_front] - _processing_data.line_distance[index_rear];
  const double error_yaw = std::atan2(error_line, _parameter.d_x);

  const double yaw_rate = _processing_data.orientate_to_line->process(0.0, -error_yaw, dt);

  // RCLCPP_INFO(get_logger(), "yaw error = %f.", error_yaw);
  // RCLCPP_INFO(get_logger(), "yaw rate output = %f.", yaw_rate);

  // Calculate error in y direction.
  const double error_y = error_line / 2.0f + _processing_data.line_distance[index_rear];

  const double vel_y = _processing_data.stay_on_line->process(0.0, -error_y, dt);

  // RCLCPP_INFO(get_logger(), "error in y direction = %f.", error_y);
  // RCLCPP_INFO(get_logger(), "velocity y = %f.", vel_y);

  // Estimate if robot is on track.
  std_msgs::msg::Bool on_track;

  on_track.data =
    std::abs(error_y) < _parameter.max_error_on_track && std::abs(error_yaw) < _parameter.max_error_yaw;
  _pub_on_track->publish(on_track);

  // Finish processing.
  _processing_data.stamp_last_processing = stamp_now;
  _processing_data.line_distance_received.fill(false);
  _processing_data.valid_line_distance.fill(false);
  _processing_data.line_distance.fill(0.0);

  geometry_msgs::msg::Twist twist_out;

  twist_out.linear.y = vel_y;
  twist_out.angular.z = yaw_rate;

  _pub_velocity->publish(twist_out);
}

std::size_t SickLineController::getBestIndex(const std::vector<std::int64_t>& group_source_ids)
{
  if (group_source_ids.empty()) {
    throw std::invalid_argument("group source ids must at least contain one id"); 
  }

  // pick up indices (at least one) for given group source ids
  std::vector<std::size_t> indices;

  for (const auto source_id : group_source_ids) {
    indices.push_back(*get_index_from_source_id(_parameter.source_ids, source_id));
  }

  // determine best sensor's index 
  std::size_t best_index = indices.front();

  for (const auto index : indices) {
    if (_processing_data.cnt_lpc[index] < _processing_data.cnt_lpc[best_index]) {
      best_index = index;
    }
  }

  return best_index;
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eduart::fleet::SickLineController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
