#include "edu_fleet/kalman_filter/extended_kalman_filter.hpp"

#include <Eigen/Dense>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <stdexcept>
#include <cstddef>
#include <iostream>

namespace eduart {
namespace fleet {
namespace kalman_filter {

ExtendedKalmanFilterBase::ExtendedKalmanFilterBase(
  std::unique_ptr<FilterModelInterface> filter_model, std::unique_ptr<AttributeVectorInterface> state_vector)
  : _model(std::move(filter_model))
  , _state(std::move(state_vector))
  , _covariance(Eigen::MatrixX<Data>::Zero(_state->get().size(), _state->get().size()))
{
  // guarantee model fits to state vector
  if (static_cast<std::size_t>(_state->get().size()) != _model->rows() || static_cast<std::size_t>(_state->get().size()) != _model->cols()) {
    throw std::invalid_argument("KalmanFilterBase: model doesn't fit to state vector.");
  }

  // initialize inner states and covariances
  // states are zero
  _state->set(Eigen::VectorX<Data>::Zero(_state->get().rows(), _state->get().cols()));
  // covariance hight so filter will fast accept first measurements
  _covariance = Eigen::MatrixX<Data>::Identity(_covariance.rows(), _covariance.cols()) * 1000.0;
}

void ExtendedKalmanFilterBase::initialize(const Eigen::VectorX<Data>& state, const Eigen::MatrixX<Data>& covariance)
{
  if (static_cast<std::size_t>(state.size()) != _model->rows() ||
      static_cast<std::size_t>(covariance.rows()) != _model->rows() ||
      static_cast<std::size_t>(covariance.cols()) != _model->cols()) {
    throw std::invalid_argument("KalmanFilterBase: given initial values doe not have the correct size.");
  }

  _state->set(state);
  _covariance = covariance;
}

void ExtendedKalmanFilterBase::process(
  const Eigen::VectorX<Data>& measurement, const Eigen::MatrixX<Data>& measurement_covariance,
  const Eigen::MatrixX<Data>& observation_matrix, const rclcpp::Time& stamp)
{
  predictToTime(_predicted_state, _predicted_covariance, stamp);

  update(
    measurement,
    measurement_covariance,
    observation_matrix,
    _predicted_state,
    _predicted_covariance
  );
}

void ExtendedKalmanFilterBase::predictToTimeAndKeep(const rclcpp::Time& stamp)
{
  predictToTime(_predicted_state, _predicted_covariance, stamp);
  _state->set(_predicted_state);
  _covariance = _predicted_covariance;

  // only get stamp if in future
  if (stamp > _state_time_stamp) {
    _state_time_stamp = stamp;
  }
}

void ExtendedKalmanFilterBase::predictToTime(
  Eigen::VectorX<Data>& predicted_state, Eigen::MatrixX<Data>& predicted_covariance, const rclcpp::Time& stamp)
{
  // calculate dt and check if it is valid
  Data dt = (stamp - _state_time_stamp).seconds();

  if (dt < 0.0) {
    // no prediction needed, filter is already before given stamp
    RCLCPP_WARN(rclcpp::get_logger("ExtendedKalmanFilterBase"), "given time stamp is in past --> no prediction");
    predicted_state = _state->get();
    predicted_covariance = _covariance;
    return;
  }
  if (dt >= _parameter.max_dt) {
    RCLCPP_WARN(rclcpp::get_logger("ExtendedKalmanFilterBase"), "limit dt = %f to %f", dt, _parameter.max_dt);
    dt = _parameter.max_dt;    
  }

  // perform state prediction
  const auto F = _model->getPredictionMatrix(*_state, dt);
  const auto Q = _model->getSystemNoiseMatrix(*_state, dt);

  predicted_state = F * _state->get();
  predicted_covariance = F * _covariance * F.transpose() + Q;
}

void ExtendedKalmanFilterBase::update(
    const Eigen::VectorX<Data>& measurement, const Eigen::MatrixX<Data>& measurement_covariance, 
    const Eigen::MatrixX<Data>& observation_matrix,
    const Eigen::VectorX<Data>& predicted_state, const Eigen::MatrixX<Data>& predicted_covariance)
{
  // \todo check if this method could work using fixed size vectors and matrices to avoid allocation
  // transform state space to measurement space
  const Eigen::VectorX<Data> predicted_state_sensor_space = observation_matrix * predicted_state;
  const Eigen::MatrixX<Data> predicated_covariance_sensor_space =
    observation_matrix * predicted_covariance * observation_matrix.transpose();

  // calculate innovations
  const Eigen::VectorX<Data> innovation = measurement - predicted_state_sensor_space;
  const Eigen::MatrixX<Data> innovation_covariance = predicated_covariance_sensor_space + measurement_covariance;

  // calculate Kalman gain
  const auto kalman_gain = predicted_covariance * observation_matrix.transpose() * innovation_covariance.inverse();
  // update states and covariance using Kalman gain
  _state->set(predicted_state + kalman_gain * innovation);
  const Eigen::MatrixX<Data> I = Eigen::MatrixX<Data>::Identity(_covariance.rows(), _covariance.cols());
  _covariance = (I - kalman_gain * observation_matrix) * predicted_covariance;
}

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
