/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"

#include <cstddef>
#include <typeinfo>
#include <vector>

namespace eduart {
namespace fleet {
namespace kalman_filter {

class AttributePackInterface
{
public:
  virtual std::vector<Attribute> attributes() const = 0;
  virtual std::size_t attributes_id() const = 0;
};

template <Attribute... Attributes>
class AttributePack : public AttributePackInterface
{
public:
  inline static constexpr std::size_t size() { return sizeof...(Attributes); }
  std::vector<Attribute> attributes() const override {
    return {{ Attributes... }};
  }

  // attribute handling methods
  template <Attribute AttributeValue>
  inline static constexpr std::size_t index() {
    static_assert(count<AttributeValue>() == 1, "given attribute only must be contained once");

    std::size_t index = 0, counter = 0;
    ((AttributeValue == Attributes ? index = counter : ++counter), ...);
    return index;
  }

  template <Attribute AttributeValue>
  inline static constexpr std::size_t count() {
    std::size_t counter = 0;
    ((AttributeValue == Attributes ? ++counter : 0), ...);
    return counter;
  }

  std::size_t attributes_id() const override {
    return typeid(AttributePack<Attributes...>).hash_code();
  }
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
