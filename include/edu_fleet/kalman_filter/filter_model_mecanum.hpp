/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"
#include "edu_fleet/kalman_filter/attribute_pack.hpp"
#include "edu_fleet/kalman_filter/filter_model.hpp"

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
    struct {
      Data jerk = 0.5 * 0.5;
      Data yaw_rate = 12.566370614 * 12.566370614; // 720° stddev
    } noise;
  };

  FilterModelMecanum(const Parameter& parameter) : _parameter(parameter) { }

  const Eigen::MatrixX<Data>& getPredictionMatrix(
    const AttributeVectorInterface& current_state, const Data dt) override;
  const Eigen::MatrixX<Data>& getSystemNoiseMatrix(
    const AttributeVectorInterface& current_state, const Data dt) override;

private:
  const Parameter _parameter;
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
