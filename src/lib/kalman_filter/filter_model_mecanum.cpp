#include "edu_fleet/kalman_filter/filter_model_mecanum.hpp"
#include "edu_fleet/kalman_filter/attribute.hpp"
#include <cstddef>
#include <stdexcept>

namespace eduart {
namespace fleet {
namespace kalman_filter {

// indices helper
static constexpr std::size_t W_POS_X  = FilterModelMecanum::attribute_pack::index<Attribute::W_POS_X>();
static constexpr std::size_t W_POS_Y  = FilterModelMecanum::attribute_pack::index<Attribute::W_POS_Y>();
static constexpr std::size_t VEL_X    = FilterModelMecanum::attribute_pack::index<Attribute::VEL_X>();
static constexpr std::size_t VEL_Y    = FilterModelMecanum::attribute_pack::index<Attribute::VEL_Y>();
static constexpr std::size_t ACC_X    = FilterModelMecanum::attribute_pack::index<Attribute::ACC_X>();
static constexpr std::size_t ACC_Y    = FilterModelMecanum::attribute_pack::index<Attribute::ACC_X>();
static constexpr std::size_t W_YAW    = FilterModelMecanum::attribute_pack::index<Attribute::W_YAW>();
static constexpr std::size_t YAW_RATE = FilterModelMecanum::attribute_pack::index<Attribute::YAW_RATE>();

const Eigen::MatrixX<Data>& FilterModelMecanum::getPredictionMatrix(
  const AttributeVectorInterface& current_state, const Data dt)
{
  // check if current state has correct type
  const auto state = dynamic_cast<attribute_vector const* const>(&current_state);

  if (state == nullptr) {
    throw std::invalid_argument("FilterModelMecanum: given state has wrong type --> cancel prediction.");
  }

  
  // calculate prediction based on current state and dt
  // NOTE: predication matrix was initialized with an I matrix at beginning
  // NOTE: all lower the diagonal is zero!
  const Data cos_phi = std::cos(state->yaw());
  const Data sin_phi = std::sin(state->yaw());

  // from p_x to ...
  // ... position
  _prediction_matrix(W_POS_X, W_POS_Y) = 1.0;
  _prediction_matrix(W_POS_X, W_POS_Y) = 0.0;
  // ... velocity
  _prediction_matrix(W_POS_X, VEL_X) =  dt * cos_phi;
  _prediction_matrix(W_POS_X, VEL_Y) = -dt * sin_phi;
  // ... acceleration
  _prediction_matrix(W_POS_X, ACC_X) =  0.5 * dt * dt * cos_phi;
  _prediction_matrix(W_POS_X, ACC_Y) = -0.5 * dt * dt * sin_phi;
  // ... yaw
  _prediction_matrix(W_POS_X, W_YAW) =
    dt * dt * (-0.5 * state->acceleration_x() * sin_phi - 0.5 * state->acceleration_y() * cos_phi) +
    dt * (-state->velocity_x() * sin_phi - state->velocity_y() * cos_phi);
  // ... yaw rate
  _prediction_matrix(W_POS_X, YAW_RATE) = 0.0;

  // from p_y to ...
  // ... position
  _prediction_matrix(W_POS_Y, W_POS_Y) = 1.0;
  // ... velocity
  _prediction_matrix(W_POS_Y, VEL_X) = dt * sin_phi;
  _prediction_matrix(W_POS_Y, VEL_Y) = dt * cos_phi;
  // ... acceleration
  _prediction_matrix(W_POS_Y, ACC_X) = 0.5 * dt * dt * sin_phi;
  _prediction_matrix(W_POS_Y, ACC_Y) = 0.5 * dt * dt * cos_phi;
  // ... yaw
  _prediction_matrix(W_POS_Y, W_YAW) =
    dt * dt * (0.5 * state->acceleration_x() * cos_phi - 0.5 * state->acceleration_y() * sin_phi) +
    dt * (state->velocity_x() * cos_phi - state->velocity_y() * sin_phi);
  // ... yaw rate
  _prediction_matrix(W_POS_Y, YAW_RATE) = 0.0;

  // from v_x to ...
  // ... velocity
  _prediction_matrix(VEL_X, VEL_X) = 1.0;
  _prediction_matrix(VEL_X, VEL_Y) = 0.0;
  // ... acceleration
  _prediction_matrix(VEL_X, ACC_X) = dt;
  _prediction_matrix(VEL_X, ACC_Y) = 0.0;
  // ... yaw
  _prediction_matrix(VEL_X, W_YAW) = 0.0;
  // ... yaw rate
  _prediction_matrix(VEL_X, YAW_RATE) = 0.0;

  // from v_y to ...
  // ... velocity
  _prediction_matrix(VEL_Y, VEL_Y) = 1.0;
  // ... acceleration
  _prediction_matrix(VEL_Y, ACC_X) = 0.0;
  _prediction_matrix(VEL_Y, ACC_Y) = dt;
  // ... yaw
  _prediction_matrix(VEL_Y, W_YAW) = 0.0;
  // ... yaw rate
  _prediction_matrix(VEL_Y, YAW_RATE) = 0.0;

  // from a_x to ...
  // ... acceleration
  _prediction_matrix(ACC_X, ACC_X) = 1.0;
  _prediction_matrix(ACC_X, ACC_Y) = 0.0;
  // ... yaw
  _prediction_matrix(ACC_X, W_YAW) = 0.0;
  // ... yaw rate
  _prediction_matrix(ACC_X, YAW_RATE) = 0.0;

  // from yaw to ...
  // ... yaw
  _prediction_matrix(W_YAW, W_YAW) = 1.0;
  // ... yaw rate
  _prediction_matrix(W_YAW, YAW_RATE) = dt;

  // from yaw rate to ...
  // ... yaw rate
  _prediction_matrix(YAW_RATE, YAW_RATE) = 1.0;

  return _prediction_matrix;
}

const Eigen::MatrixX<Data>& FilterModelMecanum::getSystemNoiseMatrix(
    const AttributeVectorInterface& current_state, const Data dt)
{
  const auto state = dynamic_cast<attribute_vector const* const>(&current_state);

  if (state == nullptr) {
    throw std::invalid_argument("FilterModelMecanum: given state has wrong type --> cancel system noise calculation.");
  }

  // calculate system noise matrix
  const Data cos_phi = std::cos(state->yaw());
  const Data sin_phi = std::sin(state->yaw());
  // clear system noise matrix before add noise parts
  _system_noise_matrix.setConstant(0.0);

  // add jerk system noise
  {
    Eigen::Vector<Data, attribute_vector::size()> noise_vector = Eigen::Vector<Data, attribute_vector::size()>::Zero();

    noise_vector[W_POS_X] = dt * dt * dt * (cos_phi - sin_phi);
    noise_vector[W_POS_Y] = dt * dt * dt * (sin_phi + cos_phi);

    noise_vector[VEL_X] = dt * dt;
    noise_vector[VEL_Y] = dt * dt;
    noise_vector[ACC_X] = dt;
    noise_vector[ACC_Y] = dt;

    _system_noise_matrix += _parameter.noise.jerk * noise_vector * noise_vector.transpose();
  }

  // yaw acceleration part
  {
    Eigen::Vector<Data, attribute_vector::size()> noise_vector = Eigen::Vector<Data, attribute_vector::size()>::Zero();

    noise_vector[W_YAW] = dt * dt;
    noise_vector[YAW_RATE] = dt;

    _system_noise_matrix += _parameter.noise.yaw_rate * noise_vector * noise_vector.transpose();    
  }

  return _system_noise_matrix;
}

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
