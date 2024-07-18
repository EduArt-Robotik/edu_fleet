/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"

#include <Eigen/Core>

#include <cstddef>

namespace eduart {
namespace fleet {
namespace kalman_filter {

template <Attribute... Attributes>
class AttributeVector
{
public:
  inline static constexpr std::size_t size() { return sizeof...(Attributes); }

  void setValue(const Eigen::Vector<Data, size()>& vector) {
    std::size_t index = 0;
    ((vector[index] = perform_post_processing<Attributes>(vector[index]), ++index), ...);
  }
  inline const Eigen::Vector<Data, size()>& getValue() const { return _data; }

private:
  Eigen::Vector<Data, size()> _data;
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
