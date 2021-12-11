#include "PID.h"
#include <cmath>

namespace {

template<typename T>
double
sign(T x)
{
  if (static_cast<double>(x) > 1e-6) {
    return 1.0;
  } else if (static_cast<double>(x) < -1e-6) {
    return -1.0;
  }
  return 0.0;
}

}

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

void
PID::Init(double Kp, double Ki, double Kd)
{
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _p_error = 0.0;
  _i_error = 0.0;
  _d_error = 0.0;
  _prev_error = std::numeric_limits<double>::nan();
}

void
PID::UpdateError(double error)
{
  if (std::isnan(_prev_error)) {
    _prev_error = error;
  }

  _p_error = error;
  _i_error += error;
  _d_error = error - _prev_error;

  _prev_error = error;

  /**
   * TODO: Update PID errors based on cte.
   */
}

double
PID::TotalError() const
{
  return _Kp * _p_error + _Ki * _i_error + _Kd * _d_error;
}

bool
PID::_NeedToClamp(double output) const
{
  double curr_error = TotalError();
  double clamped_error = _TryClamp(curr_error, _min * 0.01, _max * 0.01);

  // The value is clamped, so perhaps turn off integrator
  bool is_saturated = std::abs(curr_error - clamped_error) > 1e-6;
  // When the sign is the same, we know that integrator is still functioning
  bool is_integrating = sign(output) and sign(curr_error);
  return is_saturated and is_integrating;
}

double
PID::_TryClamp(double output, double min_delta, double max_delta) const
{
  if (output < (_min - min_delta + 1e-6)) {
    return _min - min_delta;
  } else if (output > (_max - max_delta - 1e-6)) {
    return _max - max_delta;
  }
  return output;
}
