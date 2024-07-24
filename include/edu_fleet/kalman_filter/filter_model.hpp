/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute_pack.hpp"
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

template <class>
class FilterModel;

template <Attribute... Attributes>
class FilterModel<AttributePack<Attributes...>> : public FilterModelInterface
{
public:
  using attribute_pack = AttributePack<Attributes...>;
  using attribute_vector = AttributeVector<Attributes...>;

  std::size_t rows() const override { return AttributePack<Attributes...>::size(); }
  std::size_t cols() const override { return AttributePack<Attributes...>::size(); }

protected:
  Eigen::MatrixX<Data> _prediction_matrix = Eigen::MatrixX<Data>::Identity(AttributePack<Attributes...>::size(), AttributePack<Attributes...>::size());
  Eigen::MatrixX<Data> _system_noise_matrix = Eigen::MatrixX<Data>::Zero(AttributePack<Attributes...>::size(), AttributePack<Attributes...>::size());
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
