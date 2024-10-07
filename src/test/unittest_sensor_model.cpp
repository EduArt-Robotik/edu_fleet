#include <gtest/gtest.h>

#include <edu_fleet/sensor_model/sensor_model_ros.hpp>

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_pack.hpp>

#include <sensor_msgs/msg/imu.hpp>

using eduart::fleet::sensor_model::SensorModelRos;

using eduart::fleet::kalman_filter::Attribute;
using eduart::fleet::kalman_filter::AttributePack;

TEST(sensor_mode, instantiate)
{
  using Attributes = AttributePack<Attribute::ACC_X, Attribute::ACC_Y, Attribute::YAW_RATE>;

  SensorModelRos<Attributes, sensor_msgs::msg::Imu> sensor_model("sensor_model_imu");
}
