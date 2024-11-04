#include "collision_avoidance_lidar_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

namespace eduart {
namespace fleet {

CollisionAvoidanceLidar::Parameter CollisionAvoidanceLidar::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  (void)ros_node;
  return default_parameter;
}

CollisionAvoidanceLidar::CollisionAvoidanceLidar()
  : rclcpp::Node("collision_avoidance")
  , _parameter(get_parameter({}, *this))
{
  // ROS Related
  _sub_laser_scan = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::QoS(10).best_effort(),
    std::bind(&CollisionAvoidanceLidar::callbackLaserScan, this, std::placeholders::_1)
  );
  _sub_velocity = create_subscription<geometry_msgs::msg::Twist>(
    "in/cmd_vel",
    rclcpp::QoS(10).best_effort(),
    std::bind(&CollisionAvoidanceLidar::callbackVelocity, this, std::placeholders::_1)
  );
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/cmd_vel",
    rclcpp::QoS(10).reliable()
  );

  // Preparing Processing
  _processing_data.intersection.fill(false);
  _processing_data.reduce_factor.fill(1.0f);
}

CollisionAvoidanceLidar::~CollisionAvoidanceLidar()
{

}

void CollisionAvoidanceLidar::callbackLaserScan(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg)
{

}

void CollisionAvoidanceLidar::callbackVelocity(std::shared_ptr<const geometry_msgs::msg::Twist> msg)
{

}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::CollisionAvoidanceLidar>());
  rclcpp::shutdown();

  return 0;
}