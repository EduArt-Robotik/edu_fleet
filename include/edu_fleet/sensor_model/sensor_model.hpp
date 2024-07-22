/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_pack.hpp>

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include <cstddef>

namespace eduart {
namespace fleet {
namespace sensor_model {

using kalman_filter::Data;

class SensorModelBase
{
protected:
  SensorModelBase(const std::size_t dimension);

public:
  virtual ~SensorModelBase();

protected:
  Eigen::VectorX<Data> _measurement;
  Eigen::MatrixX<Data> _measurement_covariance;
  rclcpp::Time _stamp;
};

template <class>
class SensorModel;

template <kalman_filter::Attribute... Attributes>
class SensorModel<kalman_filter::AttributePack<Attributes...>> : public SensorModelBase
                                                               , public kalman_filter::AttributePack<Attributes...>
{
public:
  SensorModel() : SensorModelBase(kalman_filter::AttributePack<Attributes...>::size()) { }
  ~SensorModel() override = default;
};

} // end namespace sensor_model
} // end namespace fleet
} // end namespace eduart
