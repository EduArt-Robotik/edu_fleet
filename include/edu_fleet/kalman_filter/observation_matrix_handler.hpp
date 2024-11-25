/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute_pack.hpp"

#include <Eigen/Core>

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <map>

namespace eduart {
namespace fleet {
namespace kalman_filter {

class ObservationMatrixHandler
{
public:
  static ObservationMatrixHandler& instance();
  const Eigen::MatrixX<Data>& matrix(const AttributePackInterface& full_set, const AttributePackInterface& sub_set);

private:
  ObservationMatrixHandler() = default;
  Eigen::MatrixX<Data> createMatrix(const std::vector<Attribute>& full_set, const std::vector<Attribute>& sub_set);

  std::map<std::size_t, std::map<std::size_t, Eigen::MatrixX<Data>>> _matrix; //> first key: full_set, second key: sub_set
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
