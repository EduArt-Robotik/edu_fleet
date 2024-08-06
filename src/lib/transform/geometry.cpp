#include "edu_fleet/transform/geometry.hpp"

namespace eduart {
namespace fleet {
namespace transform {

void do_transform(
  const std::array<double, 36>& covariance_in, std::array<double, 36>& covariance_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  
}

void do_transform(
  const geometry_msgs::msg::TwistWithCovariance& twist_in, geometry_msgs::msg::TwistWithCovariance& twist_out,
  const geometry_msgs::msg::TransformStamped& transform)
{
  
}

} // end namespace transform
} // end namespace fleet
} // end namespace eduart