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

#include <Eigen/src/Core/Matrix.h>
#include <rclcpp/time.hpp>

#include <memory>

namespace eduart {
namespace fleet {
namespace kalman_filter {

class ExtendedKalmanFilterBase
{
public:
  struct Parameter{
    Data max_dt = 10.0;
  };

protected:
  ExtendedKalmanFilterBase(
    std::unique_ptr<FilterModelInterface> model, std::unique_ptr<AttributeVectorInterface> state_vector);

public:
  void process(
    const Eigen::VectorX<Data>& measurement, const Eigen::MatrixX<Data>& measurement_covariance,
    const Eigen::MatrixX<Data>& observation_matrix, const rclcpp::Time& stamp);
  void predictToTime(
    Eigen::VectorX<Data>& predicted_state, Eigen::MatrixX<Data>& predicted_covariance, const rclcpp::Time& stamp);
  void update(
    const Eigen::VectorX<Data>& measurement, const Eigen::MatrixX<Data>& measurement_covariance, 
    const Eigen::MatrixX<Data>& observation_matrix,
    const Eigen::VectorX<Data>& predicted_state, const Eigen::MatrixX<Data>& predicted_covariance);

protected:
  const Parameter _parameter;

  std::unique_ptr<FilterModelInterface> _model;
  std::shared_ptr<AttributeVectorInterface> _state;
  Eigen::MatrixX<Data> _covariance;
  rclcpp::Time _state_time_stamp;

private:
  Eigen::VectorX<Data> _predicted_state; //> helper to speed up calculation (no reallocation required then)
  Eigen::MatrixX<Data> _predicted_covariance; //> helper to speed up calculation (no reallocation required then)
};

template <class>
class ExtendedKalmanFilter;

template <Attribute ...Attributes>
class ExtendedKalmanFilter<AttributePack<Attributes...>> : public ExtendedKalmanFilterBase
{
public:
  ExtendedKalmanFilter(std::unique_ptr<FilterModelInterface> model)
    : ExtendedKalmanFilterBase(std::move(model), std::make_unique<AttributeVector<Attributes...>>())
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
