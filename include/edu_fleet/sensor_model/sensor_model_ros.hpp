/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/sensor_model/sensor_model.hpp"
#include "edu_fleet/sensor_model/message_converting.hpp"

#include <edu_fleet/kalman_filter/attribute.hpp>
#include <edu_fleet/kalman_filter/attribute_pack.hpp>

#include <memory>

namespace eduart {
namespace fleet {
namespace sensor_model {

using kalman_filter::Attribute;
using kalman_filter::AttributePack;

template <class, class>
class SensorModelRos;

template <Attribute... Attributes, class RosMsg>
class SensorModelRos<AttributePack<Attributes...>, RosMsg> : public SensorModel<AttributePack<Attributes...>>
{
private:
  using SensorModelBase::_measurement;
  using SensorModelBase::_measurement_covariance;
  using SensorModelBase::_stamp;

public:
  SensorModelRos(const std::string& name) : _name(name) { }
  ~SensorModelRos() override = default;

  inline const std::string& name() const { return _name; }
  void process(std::shared_ptr<const RosMsg> msg) {
    message_converting<AttributePack<Attributes...>>::to_measurement(msg, _measurement);
    message_converting<AttributePack<Attributes...>>::to_covariance(msg, _measurement_covariance);
    _stamp = msg->header.stamp;
  }

private:
  std::string _name;
};

} // end namespace sensor_model
} // end namespace fleet
} // end namespace eduart
