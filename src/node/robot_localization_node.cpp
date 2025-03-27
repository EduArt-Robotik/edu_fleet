#include "robot_localization_node.hpp"
#include "edu_fleet/sensor_model/message_converting.hpp"

#include <edu_fleet/kalman_filter/extended_kalman_filter.hpp>
#include <edu_fleet/kalman_filter/filter_model_mecanum.hpp>

#include <edu_fleet/transform/geometry.hpp>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
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

RobotLocalization::Parameter RobotLocalization::get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  Parameter parameter;

  // general parameter
  ros_node.declare_parameter<std::string>("robot_name", default_parameter.robot_name);
  ros_node.declare_parameter<int>(
    "input_delay_ms", default_parameter.input_delay.nanoseconds() / 1000000);
  ros_node.declare_parameter<int>(
    "output_interval_ms", default_parameter.output_interval.nanoseconds() / 1000000);

  parameter.robot_name = ros_node.get_parameter("robot_name").as_string();
  parameter.input_delay.from_nanoseconds(
    ros_node.get_parameter("input_delay_ms").as_int() * 1000000);
  parameter.output_interval.from_nanoseconds(
    ros_node.get_parameter("output_interval_ms").as_int() * 1000000);

  // odometry parameter
  ros_node.declare_parameter<double>(
    "odometry.std_dev.min.linear", default_parameter.limit.std_dev.min.odometry.linear);
  ros_node.declare_parameter<double>(
    "odometry.std_dev.min.angular", default_parameter.limit.std_dev.min.odometry.angular);

  parameter.limit.std_dev.min.odometry.linear = ros_node.get_parameter("odometry.std_dev.min.linear").as_double();
  parameter.limit.std_dev.min.odometry.angular = ros_node.get_parameter("odometry.std_dev.min.angular").as_double();

  // imu parameter
  ros_node.declare_parameter<double>(
    "imu.std_dev.min.linear", default_parameter.limit.std_dev.min.imu.linear);
  ros_node.declare_parameter<double>(
    "imu.std_dev.min.angular", default_parameter.limit.std_dev.min.imu.angular);

  parameter.limit.std_dev.min.imu.linear = ros_node.get_parameter("imu.std_dev.min.linear").as_double();
  parameter.limit.std_dev.min.imu.angular = ros_node.get_parameter("imu.std_dev.min.angular").as_double();

  return parameter;
}

RobotLocalization::RobotLocalization()
  : rclcpp::Node("robot_localization")
  , _parameter(get_parameter(_parameter, *this))
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  , _tf_buffer(std::make_unique<tf2_ros::Buffer>(this->get_clock()))
  , _tf_listener(std::make_unique<tf2_ros::TransformListener>(*_tf_buffer))
  , _sensor_model_imu(std::make_shared<SensorModelImu>("sensor_model_imu"))
  , _sensor_model_odometry(std::make_shared<SensorModelOdometry>("sensor_model_odometry"))
  , _sensor_model_pose(std::make_shared<SensorModelPose>("sensor_model_pose"))
{
  // instantiate ROS subscriptions and publisher
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
  _pub_odometry = create_publisher<nav_msgs::msg::Odometry>(
    "localization", rclcpp::QoS(5).reliable()
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
  // _timer_publish_robot_state = rclcpp::create_timer(
  //   get_node_base_interface(), get_node_timers_interface(), get_clock(),
  //   rclcpp::Duration::from_seconds(0.01),
  //   std::bind(&RobotLocalization::checkIfStateShouldPredicted, this)
  // );
}

RobotLocalization::~RobotLocalization()
{

}

void RobotLocalization::callbackImu(std::shared_ptr<const sensor_msgs::msg::Imu> msg)
{
  // \todo check time stamp!
  try {
    // transform message into robot frame
    sensor_msgs::msg::Imu imu_transformed = *msg;
    // const auto stamp =  rclcpp::Time(msg->header.stamp) - _parameter.input_delay;    
    // const auto transform = _tf_buffer->lookupTransform(
    //   _parameter.robot_name + "/base_link", msg->header.frame_id, stamp
    // );
    // do_transform(*msg, imu_transformed, transform);

    // limit covariance
    const double var_lin_min = _parameter.limit.std_dev.min.imu.linear * _parameter.limit.std_dev.min.imu.linear;
    const double var_ang_min = _parameter.limit.std_dev.min.imu.angular * _parameter.limit.std_dev.min.imu.angular;

    imu_transformed.linear_acceleration_covariance[0] = std::max(
      imu_transformed.linear_acceleration_covariance[0], var_lin_min
    );
    imu_transformed.linear_acceleration_covariance[4] = std::max(
      imu_transformed.linear_acceleration_covariance[4], var_lin_min
    );
    imu_transformed.angular_velocity_covariance[8] = std::max(
      imu_transformed.angular_velocity_covariance[8], var_ang_min
    );

    // processing measurement data
    _sensor_model_imu->process(imu_transformed);
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::cout << "measurement:\n" << _sensor_model_imu->measurement() << std::endl;
    _kalman_filter->process(_sensor_model_imu);
    publishRobotState();
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception was thrown during processing. what = %s", ex.what());
  }
}

void RobotLocalization::callbackOdometry(std::shared_ptr<const nav_msgs::msg::Odometry> msg)
{
  // \todo check time stamp!
  try {
    // transform message into robot frame
    nav_msgs::msg::Odometry odometry_transformed = *msg;
    // const auto stamp =  rclcpp::Time(msg->header.stamp) - _parameter.input_delay;
    // const auto transform = _tf_buffer->lookupTransform(
    //   _parameter.robot_name + "/base_link", msg->child_frame_id, stamp
    // );
    // do_transform(msg->twist, odometry_transformed.twist, transform);

    // limit covariance
    const double var_lin = _parameter.limit.std_dev.min.odometry.linear * _parameter.limit.std_dev.min.odometry.linear;
    const double var_ang = _parameter.limit.std_dev.min.odometry.angular * _parameter.limit.std_dev.min.odometry.angular;

    odometry_transformed.twist.covariance[ 0] = std::max(odometry_transformed.twist.covariance[ 0], var_lin);
    odometry_transformed.twist.covariance[ 7] = std::max(odometry_transformed.twist.covariance[ 7], var_lin);
    odometry_transformed.twist.covariance[35] = std::max(odometry_transformed.twist.covariance[35], var_ang);

    std::cout << "odometry:\n";
    std::cout << "v_x = " << odometry_transformed.twist.twist.linear.x << " v_y = " << odometry_transformed.twist.twist.linear.y << std::endl;
    std::cout << "omega_z = " << odometry_transformed.twist.twist.angular.z << std::endl;

    // processing measurement data
    _sensor_model_odometry->process(odometry_transformed);
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::cout << "measurement:\n" << _sensor_model_odometry->measurement() << std::endl;    
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
    // Pose is in world coordinate system! Transform transform into base_link in world coordinate first, before applying.
    // const auto pose_transformed = _tf_buffer->transform(
    //   *msg, _parameter.robot_name + "/base_link");

    // Only uses translation! Maybe this doesn't cover all cases but however lets go!
    const auto transform = _tf_buffer->lookupTransform(
      _parameter.robot_name + "/base_link",
      msg->header.frame_id,
      msg->header.stamp
    );
    const Eigen::Vector3d translation_r(
      transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    const Eigen::Quaterniond rotation(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    const Eigen::Vector3d translation_w = rotation * translation_r;

    std::cout << "t_r:\n" << translation_r << std::endl;
    std::cout << "t_w:\n" << translation_w << std::endl;

    geometry_msgs::msg::PoseWithCovarianceStamped pose_transformed = *msg;

    pose_transformed.pose.pose.position.x -= translation_w.x();
    pose_transformed.pose.pose.position.y -= translation_w.y();
    pose_transformed.pose.pose.position.z -= translation_w.z();

    std::cout << "pose transformed:\n";
    std::cout << "x = " << pose_transformed.pose.pose.position.x << " y = " << pose_transformed.pose.pose.position.y << " z = " << pose_transformed.pose.pose.position.z << std::endl;
    std::cout << "w = " << pose_transformed.pose.pose.orientation.w << " x = " << pose_transformed.pose.pose.orientation.x << " y = " << pose_transformed.pose.pose.orientation.y << " z = " << pose_transformed.pose.pose.orientation.z << std::endl;
    _sensor_model_pose->process(pose_transformed);
    std::cout << "yaw = " << _sensor_model_pose->measurement()[2] << std::endl;

    // debug via tf
    // geometry_msgs::msg::TransformStamped t_debug;
    // t_debug.header.stamp = msg->header.stamp;
    // t_debug.header.frame_id = _parameter.robot_name + "/base_link";
    // // t_debug.header.frame_id = msg->header.frame_id;
    // t_debug.child_frame_id = "pose_debug_out";

    // t_debug.transform.translation.x = pose_transformed.pose.pose.position.x;
    // t_debug.transform.translation.y = pose_transformed.pose.pose.position.y;
    // t_debug.transform.translation.z = pose_transformed.pose.pose.position.z;

    // t_debug.transform.rotation = pose_transformed.pose.pose.orientation;
    // _tf_broadcaster->sendTransform(t_debug);

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
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  std::cout << "model stamp = " << _kalman_filter->stamp().seconds() << std::endl;
  transform.header.stamp = _kalman_filter->stamp();
  transform.header.frame_id = "map";
  transform.child_frame_id = _parameter.robot_name + "/base_footprint";

  // via tf
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

  // via odometry message
  auto odometry_msg = sensor_model::message_converting<FilterModel::attribute_pack>::to_ros(
    _kalman_filter->state().get(), _kalman_filter->covariance()
  );
  odometry_msg.header.frame_id = "map";
  odometry_msg.header.stamp = _kalman_filter->stamp();
  odometry_msg.child_frame_id = _parameter.robot_name + "/base_footprint";

  _pub_odometry->publish(odometry_msg);
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

  if ((stamp_now - _stamp_last_published) < _parameter.output_interval) {
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
  rclcpp::spin(std::make_shared<eduart::fleet::RobotLocalization>());
  rclcpp::shutdown();

  return 0;
}