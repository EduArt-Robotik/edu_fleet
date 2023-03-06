/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace fleet {

class PidController
{
public:
  struct Parameter {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double limit = 10.0;
    bool use_anti_windup = true;
  } parameter;

  PidController() = default;

  void reset();
  double operator()(const double set_point, const double feedback, const double dt);

private:
  double _e_integral = 0.0f;
  double _e_prev = 0.0f;
  double _set_point_prev = 0.0f;
};

} // end namespace fleet
} // end namespace eduart
