/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute_vector.hpp"

#include <Eigen/Core>

#include <cstddef>

namespace eduart {
namespace fleet {
namespace kalman_filter {

class FilterModelInterface
{
public:
  virtual std::size_t rows() const = 0;
  virtual std::size_t cols() const = 0;
  virtual const Eigen::MatrixX<Data>& getPredictionMatrix(const AttributeVectorInterface& current_state, const Data dt) = 0;
  virtual const Eigen::MatrixX<Data>& getSystemNoiseMatrix(const AttributeVectorInterface& current_state, const Data dt) = 0;
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
