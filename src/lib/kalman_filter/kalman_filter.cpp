#include "edu_fleet/kalman_filter/kalman_filter.hpp"

namespace eduart {
namespace fleet {
namespace kalman_filter {

KalmanFilterBase::KalmanFilterBase(
  std::unique_ptr<FilterModelInterface> filter_model, std::unique_ptr<AttributeVectorInterface> state_vector)
{

}

void KalmanFilterBase::process(
  const Eigen::VectorX<Data>& measurement, const Eigen::MatrixX<Data>& observation_matrix, const rclcpp::Time& stamp)
{

}

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
