/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"
#include "edu_fleet/kalman_filter/attribute_pack.hpp"
#include "edu_fleet/kalman_filter/filter_model.hpp"

#include <cmath>

namespace eduart {
namespace fleet {
namespace kalman_filter {

class FilterModelMecanum : public FilterModel<AttributePack<Attribute::W_POS_X,
                                                            Attribute::W_POS_Y,
                                                            Attribute::VEL_X,
                                                            Attribute::VEL_Y,
                                                            Attribute::ACC_X,
                                                            Attribute::ACC_Y,
                                                            Attribute::W_YAW,
                                                            Attribute::YAW_RATE>>
{
public:
  struct Parameter {
    enum class ModelType {
      PUSH_AND_ROTATE,
      ROTATE_AND_PUSH
    };
    ModelType mode_type = ModelType::PUSH_AND_ROTATE;
    struct {
      Data acceleration = 5.5 * 5.5;
      Data yaw_rate = M_PI_4 * M_PI_4; // 45° stddev
    } noise;
  };

  FilterModelMecanum(const Parameter& parameter) : _parameter(parameter) { }

  const Eigen::MatrixX<Data>& getPredictionMatrix(
    const AttributeVectorInterface& current_state, const Data dt) override;
  const Eigen::MatrixX<Data>& getSystemNoiseMatrix(
    const AttributeVectorInterface& current_state, const Data dt) override;

private:
  void calculatePredictionMatrixPushAndRotate(const AttributeVectorInterface& current_state, const Data dt);
  void calculatePredictionMatrixRotateAndPush(const AttributeVectorInterface& current_state, const Data dt);
  void calculateSystemNoiseMatrixPushAndRotate(const AttributeVectorInterface& current_state, const Data dt);
  void calculateSystemNoiseMatrixRotateAndPush(const AttributeVectorInterface& current_state, const Data dt);

  const Parameter _parameter;
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
