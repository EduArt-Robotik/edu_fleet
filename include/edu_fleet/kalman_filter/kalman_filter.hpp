/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"
#include "edu_fleet/kalman_filter/attribute_pack.hpp"
#include "edu_fleet/kalman_filter/filter_model.hpp"
#include "edu_fleet/kalman_filter/attribute_vector.hpp"

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include <memory>

namespace eduart {
namespace fleet {
namespace kalman_filter {

class KalmanFilterBase
{
protected:
  KalmanFilterBase(std::unique_ptr<FilterModelInterface> model, std::unique_ptr<AttributeVectorInterface> state_vector);

public:
  void process(
    const Eigen::VectorX<Data>& measurement, const Eigen::MatrixX<Data>& observation_matrix, const rclcpp::Time& stamp);
  void predictToTime(const rclcpp::Time& stamp);
  void update(const Eigen::VectorX<Data>& measurement, const Eigen::MatrixX<Data>& observation_matrix);

protected:
  std::unique_ptr<FilterModelInterface> _model;
  std::shared_ptr<AttributeVectorInterface> _state;
  Eigen::MatrixX<Data> _covariance;
};

template <class>
class KalmanFilter;

template <Attribute ...Attributes>
class KalmanFilter<AttributePack<Attributes...>> : public KalmanFilterBase
{
public:
  KalmanFilter(std::unique_ptr<FilterModelInterface> model)
    : KalmanFilterBase(std::move(model), std::make_unique<AttributeVector<Attributes...>>())
  { }

  const AttributeVector<Attributes...>& state() const {
    // pointer type is well known so use static cast instead of dynamic cast
    return std::static_pointer_cast<AttributeVector<Attributes...>>(_state);
  }
  const Eigen::MatrixX<Data> covariance() const {
    return _covariance;
  }
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
