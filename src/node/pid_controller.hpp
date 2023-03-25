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
    double input_filter_weight = 1.0;
    bool use_anti_windup = true;
  } parameter;

  PidController() = default;

  void reset();
  double operator()(const double set_point, const double feedback, const double dt);

private:
  double _e_integral = 0.0;
  double _e_prev = 0.0;
  double _set_point_prev = 0.0;
  double _previous_feedback = 0.0;
};

} // end namespace fleet
} // end namespace eduart
