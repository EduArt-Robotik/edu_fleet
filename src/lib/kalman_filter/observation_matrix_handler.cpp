#include "edu_fleet/kalman_filter/observation_matrix_handler.hpp"
#include "edu_fleet/kalman_filter/attribute.hpp"
#include <cstddef>

namespace eduart {
namespace fleet {
namespace kalman_filter {

ObservationMatrixHandler& ObservationMatrixHandler::instance()
{
  static ObservationMatrixHandler _handler;
  return _handler;
}

const Eigen::MatrixX<Data>& ObservationMatrixHandler::matrix(
  const AttributePackInterface& full_set, const AttributePackInterface& sub_set)
{
  // full set
  auto search_full_set = _matrix.find(full_set.attributes_id());

  if (search_full_set == _matrix.end()) {
    // no entry --> create matrix
    std::map<std::size_t, Eigen::MatrixX<Data>> entry;
    entry.insert(
      std::pair(sub_set.attributes_id(), createMatrix(full_set.attributes(), sub_set.attributes()))
    );
    _matrix[full_set.attributes_id()] = entry;

    return _matrix[full_set.attributes_id()][sub_set.attributes_id()];
  }

  // sub set
  auto search_sub_set = search_full_set->second.find(sub_set.attributes_id());

  if (search_sub_set == search_full_set->second.end()) {
    // no entry --> create matrix
    search_full_set->second[sub_set.attributes_id()] = createMatrix(full_set.attributes(), sub_set.attributes());
    return search_full_set->second[sub_set.attributes_id()];
  }

  return search_sub_set->second;
}

Eigen::MatrixX<Data> ObservationMatrixHandler::createMatrix(
  const std::vector<Attribute>& full_set, const std::vector<Attribute>& sub_set)
{
  Eigen::MatrixX<Data> matrix(sub_set.size(), full_set.size());

  for (std::size_t row = 0; row < static_cast<std::size_t>(matrix.rows()); ++row) {
    for (std::size_t col = 0; col < static_cast<std::size_t>(matrix.cols()); ++col) {
      matrix(row, col) = full_set[col] == sub_set[row] ? static_cast<Data>(1) : static_cast<Data>(0);
    }
  }

  return matrix;
}

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
