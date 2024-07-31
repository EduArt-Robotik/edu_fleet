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

class SensorModelBase : public kalman_filter::AttributePackInterface
{
protected:
  SensorModelBase(const std::size_t dimension);

public:
  virtual ~SensorModelBase();

  inline const Eigen::VectorX<Data>& measurement() const { return _measurement; }
  inline const Eigen::MatrixX<Data>& covariance() const { return _measurement_covariance; }
  inline const rclcpp::Time& stamp() const { return _stamp; }

protected:
  Eigen::VectorX<Data> _measurement;
  Eigen::MatrixX<Data> _measurement_covariance;
  rclcpp::Time _stamp;
};

template <class>
class SensorModel;

template <kalman_filter::Attribute... Attributes>
class SensorModel<kalman_filter::AttributePack<Attributes...>> : public SensorModelBase
{
public:
  SensorModel() : SensorModelBase(kalman_filter::AttributePack<Attributes...>::size()) { }
  ~SensorModel() override = default;

  std::vector<kalman_filter::Attribute> attributes() const override {
    return kalman_filter::AttributePack<Attributes...>().attributes();
  }
  std::size_t attributes_id() const override {
    return kalman_filter::AttributePack<Attributes...>().attributes_id();
  }
};

} // end namespace sensor_model
} // end namespace fleet
} // end namespace eduart
