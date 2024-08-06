#include "robot_localization_node.hpp"

#include <edu_fleet/kalman_filter/extended_kalman_filter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>

#include <edu_fleet/transform/geometry.hpp>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/qos.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <memory>
#include <exception>

namespace eduart {
namespace fleet {

using transform::do_transform;
using kalman_filter::FilterModelMecanum;
using kalman_filter::ExtendedKalmanFilter;

RobotLocalization::Parameter RobotLocalization::get_parameter(
  const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  (void)name;
  (void)default_parameter;
  (void)ros_node;
  Parameter parameter;

  return parameter;
}

RobotLocalization::RobotLocalization(const Parameter& parameter)
  : rclcpp::Node("robot_localization")
  , _parameter(parameter)
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  , _tf_buffer(std::make_unique<tf2_ros::Buffer>(this->get_clock()))
  , _tf_listener(std::make_unique<tf2_ros::TransformListener>(*_tf_buffer))
  , _sensor_model_imu(std::make_shared<SensorModelImu>("sensor_model_imu"))
  , _sensor_model_odometry(std::make_shared<SensorModelOdometry>("sensor_model_odometry"))
  , _sensor_model_pose(std::make_shared<SensorModelPose>("sensor_model_pose"))
{
  // instantiate ROS subscriptions
  _sub_imu = create_subscription<sensor_msgs::msg::Imu>(
    "imu",
    rclcpp::QoS(5).best_effort(),
    [this](std::shared_ptr<const sensor_msgs::msg::Imu> msg) { callbackImu(msg); }
  );
  _sub_odometry = create_subscription<nav_msgs::msg::Odometry>(
    "odometry",
    rclcpp::QoS(5).best_effort(),
    [this](std::shared_ptr<const nav_msgs::msg::Odometry> msg) { callbackOdometry(msg); }
  );
  _sub_pose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose",
    rclcpp::QoS(5).best_effort(),
    [this](std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg) { callbackPose(msg); }
  );

  // bring up services
  _srv_reset = create_service<std_srvs::srv::Trigger>(
    "robot_localization/reset",
    std::bind(&RobotLocalization::callbackReset, this, std::placeholders::_1, std::placeholders::_2)
  );

  // instantiate Kalman filter
  auto filter_model = std::make_unique<FilterModelMecanum>(_parameter.filter_parameter);
  _kalman_filter = std::make_unique<ExtendedKalmanFilter<FilterModelMecanum::attribute_pack>>(std::move(filter_model));

  // start timer for publishing state
  _timer_publish_robot_state = rclcpp::create_timer(
    get_node_base_interface(), get_node_timers_interface(), get_clock(),
    rclcpp::Duration::from_seconds(0.01),
    std::bind(&RobotLocalization::checkIfStateShouldPredicted, this)
  );
}

RobotLocalization::~RobotLocalization()
{

}

void RobotLocalization::callbackImu(std::shared_ptr<const sensor_msgs::msg::Imu> msg)
{
  // \todo check time stamp!
  try {
    // sensor_msgs::msg::Imu imu_transformed = _tf_buffer->transform(
    //   *msg, _parameter.robot_name + "/base_link");

    // _sensor_model_imu->process(imu_transformed);
    // _kalman_filter->process(_sensor_model_imu);
    // publishRobotState();
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception was thrown during processing. what = %s", ex.what());
  }
}

void RobotLocalization::callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
{
  // \todo check time stamp!
  try {
    nav_msgs::msg::Odometry odometry_transformed;
    const auto transform = _tf_buffer->lookupTransform(
      _parameter.robot_name + "/base_link", msg->child_frame_id, msg->header.stamp
    );
    do_transform(msg->twist.twist, odometry_transformed.twist.twist, transform);

    _sensor_model_odometry->process(odometry_transformed);
    _kalman_filter->process(_sensor_model_odometry);
    publishRobotState();
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception was thrown during processing. what = %s", ex.what());
  }
}

void RobotLocalization::callbackPose(std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> msg)
{
  // \todo check time stamp!
  try {
    const auto pose_transformed = _tf_buffer->transform(
      *msg, _parameter.robot_name + "/base_link");

    _sensor_model_pose->process(pose_transformed);
    _kalman_filter->process(_sensor_model_pose);
    publishRobotState();
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception was thrown during processing. what = %s", ex.what());
  }
}

void RobotLocalization::callbackReset(
  std::shared_ptr<const std_srvs::srv::Trigger_Request> request, std::shared_ptr<std_srvs::srv::Trigger_Response> response)
{
  (void)request;
  const std::size_t dimension = FilterModelMecanum::attribute_pack::size();

  _kalman_filter->initialize(
    Eigen::Vector<kalman_filter::Data, dimension>::Zero(),
    Eigen::Matrix<kalman_filter::Data, dimension, dimension>::Identity() * 1000.0
  );
 
  response->success = true;
  response->message = "fleet localization reset";
}

void RobotLocalization::publishRobotState()
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = _kalman_filter->stamp();
  transform.header.frame_id = "map";
  transform.child_frame_id = _parameter.robot_name + "/base_footprint";

  // transform
  const auto& state = _kalman_filter->state();

  // position
  transform.transform.translation.x = state.x();
  transform.transform.translation.y = state.y();

  // orientation  
  Eigen::Quaterniond orientation(Eigen::AngleAxisd(state.yaw(), Eigen::Vector3d::UnitZ()));
  transform.transform.rotation.w = orientation.w();
  transform.transform.rotation.x = orientation.x();
  transform.transform.rotation.y = orientation.y();
  transform.transform.rotation.z = orientation.z();

  _tf_broadcaster->sendTransform(transform);
  _stamp_last_published = get_clock()->now();
}

void RobotLocalization::checkIfStateShouldPredicted()
{
  if (_kalman_filter->stamp().seconds() == 0.0) {
    // never a measurement received --> do nothing
    return;
  }

  // check last publishing stamp
  const auto stamp_now = get_clock()->now();

  if ((stamp_now - _stamp_last_published) < rclcpp::Duration::from_seconds(0.01)) {
    // last publishing had just been
    return;
  }

  std::cout << "stamp now = " << stamp_now.seconds() << std::endl;
  std::cout << "model stamp = " << _kalman_filter->stamp().seconds() << std::endl;

  // predict state to stamp now and keep states
  _kalman_filter->predictToTimeAndKeep(stamp_now);
  // publishing predicted states
  publishRobotState();
}

std::string RobotLocalization::getFrameIdPrefix() const 
{
  // remove slash at the beginning
  std::string frame_id_prefix(get_effective_namespace().begin() + 1, get_effective_namespace().end());
  // add slash at the end if it is missing
  if (frame_id_prefix.back() != '/') {
    frame_id_prefix.push_back('/');
  }
  return frame_id_prefix;
}

} // end namespace fleet
} // end namespace eduart


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<eduart::fleet::RobotLocalization>(eduart::fleet::RobotLocalization::Parameter{}));
  rclcpp::shutdown();

  return 0;
}