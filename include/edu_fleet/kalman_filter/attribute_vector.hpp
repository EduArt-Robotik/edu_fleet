/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <stdexcept>

namespace eduart {
namespace fleet {
namespace kalman_filter {
namespace impl {

// top level class with data container
class AttributeVectorData
{
public:
  const Eigen::VectorX<Data>& data() const { return _data; }

protected:
  Eigen::VectorX<Data> _data;
};

// data accessors that add methods to access to
template <std::size_t Index, Attribute...>
class AttributeVectorAccessor;

template <std::size_t Index>
class AttributeVectorAccessor<Index> : public AttributeVectorData {
protected:
  using AttributeVectorData::_data;
};

// dummy specialization for attributes that are not specialized yet
template <std::size_t Index, Attribute Head, Attribute... Tail>
class AttributeVectorAccessor<Index, Head, Tail...> : public AttributeVectorAccessor<Index + 1, Tail...> {
protected:
  using AttributeVectorAccessor<Index + 1, Tail...>::_data;
};

// specialization for each attribute
#define SPECIALIZE_ATTRIBUTE_ACCESS(name, attribute) \
template <std::size_t Index, Attribute... Tail> \
class AttributeVectorAccessor<Index, attribute, Tail...> : public AttributeVectorAccessor<Index + 1, Tail...> { \
protected: \
  using AttributeVectorAccessor<Index + 1, Tail...>::_data; \
\
public: \
  inline Data name() const { return _data[Index]; } \
  inline Data& name() { return _data[Index]; } \
}; \

SPECIALIZE_ATTRIBUTE_ACCESS(x, Attribute::W_POS_X)
SPECIALIZE_ATTRIBUTE_ACCESS(y, Attribute::W_POS_Y)
SPECIALIZE_ATTRIBUTE_ACCESS(velocity, Attribute::VEL)
SPECIALIZE_ATTRIBUTE_ACCESS(velocity_x, Attribute::VEL_X)
SPECIALIZE_ATTRIBUTE_ACCESS(velocity_y, Attribute::VEL_Y)
SPECIALIZE_ATTRIBUTE_ACCESS(acceleration, Attribute::ACC)
SPECIALIZE_ATTRIBUTE_ACCESS(acceleration_x, Attribute::ACC_X)
SPECIALIZE_ATTRIBUTE_ACCESS(acceleration_y, Attribute::ACC_Y)
SPECIALIZE_ATTRIBUTE_ACCESS(roll, Attribute::W_ROLL)
SPECIALIZE_ATTRIBUTE_ACCESS(pitch, Attribute::W_PITCH)
SPECIALIZE_ATTRIBUTE_ACCESS(yaw, Attribute::W_YAW)
SPECIALIZE_ATTRIBUTE_ACCESS(roll_rate, Attribute::ROLL_RATE)
SPECIALIZE_ATTRIBUTE_ACCESS(pitch_rate, Attribute::PITCH_RATE)
SPECIALIZE_ATTRIBUTE_ACCESS(yaw_rate, Attribute::YAW_RATE)

} // end namespace

class AttributeVectorInterface
{
public:
  virtual void set(const Eigen::VectorX<Data>& vector) = 0;
  virtual const Eigen::VectorX<Data>& get() const = 0;
};

template <Attribute... Attributes>
class AttributeVector : public AttributeVectorInterface
                      , public impl::AttributeVectorAccessor<0, Attributes...>
{
protected:
  using impl::AttributeVectorAccessor<0, Attributes...>::_data;

public:
  AttributeVector() {
    _data = Eigen::VectorX<Data>::Zero(size());
  }

  inline static constexpr std::size_t size() { return sizeof...(Attributes); }

  void set(const Eigen::VectorX<Data>& vector) override {
    // grantee vector sizes match
    if (vector.size() != size()) {
      throw std::invalid_argument("AttributeVector: given vector size doesn't fit to attribute vector.");
    }

    // perform post processing on each vector element
    std::size_t index = 0;
    ((_data[index] = perform_post_processing<Attributes>()(vector[index]), ++index), ...);
  }
  const Eigen::VectorX<Data>& get() const override { return _data; }

  // attribute handling methods
  template <Attribute AttributeValue>
  inline static constexpr std::size_t index() {
    static_assert(count<AttributeValue>() == 1, "given attribute only must be contained once");

    std::size_t index = 0, counter = 0;
    ((AttributeValue == Attributes ? index = counter : ++counter), ...);
    return index;
  }

private:
  template <Attribute AttributeValue>
  inline static constexpr std::size_t count() {
    std::size_t counter = 0;
    ((AttributeValue == Attributes ? ++counter : 0), ...);
    return counter;
  }
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
