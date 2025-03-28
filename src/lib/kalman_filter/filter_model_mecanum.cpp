#include "edu_fleet/kalman_filter/filter_model_mecanum.hpp"
#include "edu_fleet/kalman_filter/attribute.hpp"

#include <cstddef>
#include <iostream>
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
static constexpr std::size_t ACC_Y    = FilterModelMecanum::attribute_pack::index<Attribute::ACC_Y>();
static constexpr std::size_t W_YAW    = FilterModelMecanum::attribute_pack::index<Attribute::W_YAW>();
static constexpr std::size_t YAW_RATE = FilterModelMecanum::attribute_pack::index<Attribute::YAW_RATE>();

const Eigen::MatrixX<Data>& FilterModelMecanum::getPredictionMatrix(
  const AttributeVectorInterface& current_state, const Data dt)
{
  switch (_parameter.mode_type)
  {
    case Parameter::ModelType::PUSH_AND_ROTATE:
      calculatePredictionMatrixPushAndRotate(current_state, dt);
      break;

    case Parameter::ModelType::ROTATE_AND_PUSH:
      calculatePredictionMatrixRotateAndPush(current_state, dt);
      break;

    default:
      throw std::invalid_argument("FilterModelMecanum: unsupported model type.");
  }

  return _prediction_matrix;
}

void FilterModelMecanum::calculatePredictionMatrixPushAndRotate(
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
  _prediction_matrix(W_POS_X, W_YAW) = 0.0;
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
  _prediction_matrix(W_POS_Y, W_YAW) = 0.0;
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

  // from a_y to ...
  // ... acceleration
  _prediction_matrix(ACC_Y, ACC_Y) = 1.0;
  // ... yaw
  _prediction_matrix(ACC_Y, W_YAW) = 0.0;
  // ... yaw rate
  _prediction_matrix(ACC_Y, YAW_RATE) = 0.0;  

  // from yaw to ...
  // ... yaw
  _prediction_matrix(W_YAW, W_YAW) = 1.0;
  // ... yaw rate
  _prediction_matrix(W_YAW, YAW_RATE) = dt;

  // from yaw rate to ...
  // ... yaw rate
  _prediction_matrix(YAW_RATE, YAW_RATE) = 1.0;
}

void FilterModelMecanum::calculatePredictionMatrixRotateAndPush(
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
  const Data cos_phi = std::cos(state->yaw() + state->yaw_rate() * dt);
  const Data sin_phi = std::sin(state->yaw() + state->yaw_rate() * dt);

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
  _prediction_matrix(W_POS_X, W_YAW) = 0.0;
  // ... yaw rate
  _prediction_matrix(W_POS_X, YAW_RATE) = 
    -dt * dt * (0.5 * dt * (state->acceleration_x() * sin_phi + state->acceleration_y() * cos_phi) +
                state->velocity_x() * sin_phi + state->velocity_y() * cos_phi);

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
  _prediction_matrix(W_POS_Y, W_YAW) = 0.0;
  // ... yaw rate
  _prediction_matrix(W_POS_Y, YAW_RATE) =
    dt * dt * (0.5 * dt * (state->acceleration_x() * cos_phi - state->acceleration_y() * sin_phi) +
               state->velocity_x() * cos_phi - state->velocity_y() * sin_phi);

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

  // from a_y to ...
  // ... acceleration
  _prediction_matrix(ACC_Y, ACC_Y) = 1.0;
  // ... yaw
  _prediction_matrix(ACC_Y, W_YAW) = 0.0;
  // ... yaw rate
  _prediction_matrix(ACC_Y, YAW_RATE) = 0.0;  

  // from yaw to ...
  // ... yaw
  _prediction_matrix(W_YAW, W_YAW) = 1.0;
  // ... yaw rate
  _prediction_matrix(W_YAW, YAW_RATE) = dt;

  // from yaw rate to ...
  // ... yaw rate
  _prediction_matrix(YAW_RATE, YAW_RATE) = 1.0;
}

const Eigen::MatrixX<Data>& FilterModelMecanum::getSystemNoiseMatrix(
    const AttributeVectorInterface& current_state, const Data dt)
{
  if (_parameter.mode_type == FilterModelMecanum::Parameter::ModelType::PUSH_AND_ROTATE) {
    calculateSystemNoiseMatrixPushAndRotate(current_state, dt);  
  }
  else {
    calculateSystemNoiseMatrixRotateAndPush(current_state, dt);
  }

  return _system_noise_matrix;
}

void FilterModelMecanum::calculateSystemNoiseMatrixPushAndRotate(
  const AttributeVectorInterface& current_state, const Data dt)
{
  const auto state = dynamic_cast<attribute_vector const* const>(&current_state);

  if (state == nullptr) {
    throw std::invalid_argument("FilterModelMecanum: given state has wrong type --> cancel system noise calculation.");
  }

  // calculate system noise matrix
  // const Data cos_phi = std::cos(state->yaw());
  // const Data sin_phi = std::sin(state->yaw());
  // clear system noise matrix before add noise parts
  _system_noise_matrix.setConstant(0.0);

  // add acceleration system noise
  {
    Eigen::Vector<Data, attribute_vector::size()> noise_vector = Eigen::Vector<Data, attribute_vector::size()>::Zero();

    noise_vector[W_POS_X] = 0.5 * dt * dt * dt;// * (cos_phi - sin_phi);
    noise_vector[W_POS_Y] = 0.5 * dt * dt * dt;// * (sin_phi + cos_phi);

    noise_vector[VEL_X] = dt * dt;
    noise_vector[VEL_Y] = dt * dt;
    noise_vector[ACC_X] = 1.0 * dt;
    noise_vector[ACC_Y] = 1.0 * dt;

    _system_noise_matrix += _parameter.noise.acceleration * noise_vector * noise_vector.transpose();
    std::cout << "Q_a:\n" << _system_noise_matrix << std::endl;
  }

  // add yaw rate system noise
  {
    Eigen::Vector<Data, attribute_vector::size()> noise_vector = Eigen::Vector<Data, attribute_vector::size()>::Zero();

    noise_vector[W_YAW] = dt;
    noise_vector[YAW_RATE] = 1.0;

    _system_noise_matrix += _parameter.noise.yaw_rate * noise_vector * noise_vector.transpose();    
  }
}

void FilterModelMecanum::calculateSystemNoiseMatrixRotateAndPush(
  const AttributeVectorInterface& current_state, const Data dt)
{
  (void)current_state;
  (void)dt;
  throw std::runtime_error("FilterModelMecanum: not implemented method called");
}

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
