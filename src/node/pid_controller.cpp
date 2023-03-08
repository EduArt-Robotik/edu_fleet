#include "pid_controller.hpp"

namespace eduart {
namespace fleet {

void PidController::reset()
{
  _e_integral = 0.0;
  _e_prev = 0.0;
  _set_point_prev = 0.0;
}

double PidController::operator()(const double set_point, const double feedback, const double dt)
{
	const double e = set_point - feedback;

	_e_integral += e * dt;

	double fy = parameter.kp * e // KP
	         + parameter.ki * _e_integral // KI
					 + parameter.kd * (e - _e_prev) / dt; // KD

	if (fy > parameter.limit) {
		fy = parameter.limit;
	
		if(parameter.use_anti_windup == true) {
			_e_integral -= e * dt;
		}
	}
	else if(fy < -parameter.limit) {
		fy = -parameter.limit;

		if(parameter.use_anti_windup == true) {
			_e_integral -= e * dt;
		}
	}

	_e_prev = e;
  _set_point_prev = set_point;
	return fy;  
}

} // end namespace fleet
} // end namespace eduart
