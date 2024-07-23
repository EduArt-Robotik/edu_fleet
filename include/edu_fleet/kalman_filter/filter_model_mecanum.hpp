/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"
#include "edu_fleet/kalman_filter/attribute_pack.hpp"
#include "edu_fleet/kalman_filter/filter_model.hpp"

#include <cstddef>

namespace eduart {
namespace fleet {
namespace kalman_filter {

template <class>
class FilterModelMecanum;

template <Attribute... Attributes>
class FilterModelMecanum<AttributePack<Attributes...>> : public FilterModelInterface
{
public:
  std::size_t rows() const override { return AttributePack<Attributes...>::size(); }
  std::size_t cols() const override { return AttributePack<Attributes...>::size(); }

  
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
