#include "edu_fleet/sensor_model/sensor_model.hpp"

namespace eduart {
namespace fleet {
namespace sensor_model {

SensorModelBase::SensorModelBase(const std::size_t dimension)
  : _measurement(Eigen::VectorX<Data>::Zero(dimension))
  , _measurement_covariance(Eigen::MatrixX<Data>::Identity(dimension, dimension) * 1000.0)
{

}

SensorModelBase::~SensorModelBase()
{
  
}

} // end namespace sensor_model
} // end namespace fleet
} // end namespace eduart
