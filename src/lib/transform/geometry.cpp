#include "edu_fleet/transform/geometry.hpp"

#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>

namespace eduart {
namespace fleet {
namespace transform {

void do_transform(
  const std::array<double, 36>& covariance_in, std::array<double, 36>& covariance_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  // transform --> rotate linear velocity covariance
  const auto& rotation = transform.transform.rotation;
  const Eigen::Quaterniond R(rotation.w, rotation.x, rotation.y, rotation.z);
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>, 0, Eigen::OuterStride<6>> in_cov_linear(
    covariance_in.data());
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>, 0, Eigen::OuterStride<6>> out_cov_linear(
    covariance_out.data());
  
  out_cov_linear = R * in_cov_linear * R.inverse();
}

void do_transform(
  const std::array<double, 9>& covariance_in, std::array<double, 9>& covariance_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  // transform --> rotate linear velocity covariance
  const auto& rotation = transform.transform.rotation;
  const Eigen::Quaterniond R(rotation.w, rotation.x, rotation.y, rotation.z);
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> in_cov_linear(
    covariance_in.data());
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> out_cov_linear(
    covariance_out.data());
  
  out_cov_linear = R * in_cov_linear * R.inverse();
}

void do_transform(
  const geometry_msgs::msg::TwistWithCovariance& twist_in, geometry_msgs::msg::TwistWithCovariance& twist_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  // only rotation needed for transforming twist message
  const auto& rotation = transform.transform.rotation;
  const Eigen::Quaterniond R(rotation.w, rotation.x, rotation.y, rotation.z);
  const Eigen::Transform<double, 3, Eigen::Affine> T(R);

  const Eigen::Vector3d twist_transformed = T * Eigen::Vector3d(
    twist_in.twist.linear.x, twist_in.twist.linear.y, twist_in.twist.linear.z);

  twist_out.twist.linear.x = twist_transformed.x();
  twist_out.twist.linear.y = twist_transformed.y();
  twist_out.twist.linear.z = twist_transformed.z();

  // transform covariances
  do_transform(twist_in.covariance, twist_out.covariance, transform);
}

void do_transform(
  const sensor_msgs::msg::Imu& imu_in, sensor_msgs::msg::Imu& imu_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  // header
  imu_out.header.stamp = imu_in.header.stamp;
  imu_out.header.frame_id = transform.header.frame_id;

  // data
  do_rotate(imu_in, imu_out, transform);
  do_translate(imu_out, imu_out, transform);
}

// To transform IMU data from a sensor mounted off-center to the center of the robot, you'll need to apply a coordinate 
// transformation. This process involves compensating for both the linear and angular effects of the offset mounting. 
// Here's a step-by-step approach:

// 1. Define the offset:
//    First, determine the exact position of the IMU relative to the robot's center. Let's call this offset vector r = (rx, ry, rz).

// 2. Linear acceleration transformation:
//    The linear acceleration at the robot's center (ac) is related to the acceleration measured by the off-center IMU (am) by:

//    ac = am - (ω × (ω × r)) - (α × r)

//    Where:
//    - ω is the angular velocity vector
//    - α is the angular acceleration vector
//    - × denotes the cross product

// 3. Angular velocity:
//    The angular velocity remains the same regardless of the mounting position:

//    ωc = ωm

// 4. Orientation:
//    The orientation (often represented as quaternions or Euler angles) also remains the same:

//    qc = qm

// 5. Implementation steps:
//    a. Measure acceleration (am), angular velocity (ωm), and orientation (qm) from the IMU.
//    b. Calculate angular acceleration (α) by differentiating angular velocity over time.
//    c. Apply the linear acceleration transformation formula.
//    d. Use the unchanged angular velocity and orientation.
void do_translate(
  const sensor_msgs::msg::Imu& imu_in, sensor_msgs::msg::Imu& imu_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  // extract needed values
  const auto& translation = transform.transform.translation;
  const Eigen::Vector3d r(translation.x, translation.y, translation.z);

  const auto& angular_velocity = imu_in.angular_velocity;
  const Eigen::Vector3d omega(angular_velocity.x, angular_velocity.y, angular_velocity.z);

  const auto& linear_acceleration = imu_in.linear_acceleration;
  const Eigen::Vector3d am(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);

  // transform linear acceleration
  const Eigen::Vector3d ac = am - (omega.cross(omega.cross(r))); // \todo maybe add the part with the alpha here

  imu_out.linear_acceleration.x = ac.x();
  imu_out.linear_acceleration.y = ac.y();
  imu_out.linear_acceleration.z = ac.z();

  imu_out.linear_acceleration_covariance = imu_in.linear_acceleration_covariance;

  // transform angular velocity
  imu_out.angular_velocity = imu_in.angular_velocity;

  imu_out.angular_velocity_covariance = imu_in.angular_velocity_covariance;

  // transform orientation
  imu_out.orientation = imu_in.orientation;

  imu_out.orientation_covariance = imu_in.orientation_covariance;
}

void do_rotate(
  const sensor_msgs::msg::Imu& imu_in, sensor_msgs::msg::Imu& imu_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  // extract needed values  
  const auto& rotation = transform.transform.rotation;
  const Eigen::Quaterniond R(rotation.w, rotation.x, rotation.y, rotation.z);

  const Eigen::Vector3d omega_m(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
  const Eigen::Vector3d a_m(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
  const Eigen::Quaterniond orientation_m(
    imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);

  // rotate linear acceleration
  const Eigen::Vector3d a_t = R * a_m;

  imu_out.linear_acceleration.x = a_t.x();
  imu_out.linear_acceleration.y = a_t.y();
  imu_out.linear_acceleration.z = a_t.z();

  do_transform(
    imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, transform
  );

  // rotate angular velocity
  const Eigen::Vector3d omega_t = R * omega_m;

  imu_out.angular_velocity.x = omega_t.x();
  imu_out.angular_velocity.y = omega_t.y();
  imu_out.angular_velocity.z = omega_t.z();

  // rotate orientation
  const Eigen::Quaterniond orientation_t = R * orientation_m;

  imu_out.orientation.w = orientation_t.w();
  imu_out.orientation.x = orientation_t.x();
  imu_out.orientation.y = orientation_t.y();
  imu_out.orientation.z = orientation_t.z();
}

} // end namespace transform
} // end namespace fleet
} // end namespace eduart