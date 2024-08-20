/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace fleet {
namespace controller {

class ControllerInterface
{
protected:
  ControllerInterface() = default;

public:
  virtual ~ControllerInterface() = default;

  virtual double process(const double set_point, const double feedback, const double dt) = 0;
  virtual void reset() = 0;
};

} // end namespace controller
} // end namespace fleet
} // end namespace eduart
