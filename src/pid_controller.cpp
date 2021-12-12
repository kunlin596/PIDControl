#include "pid_controller.h"
#include <array>
#include <cmath>
#include <iostream>

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

namespace controller {

void
PID::Init(double Kp, double Ki, double Kd)
{
  _params = { Kp, Ki, Kd };
  _errors = { 0.0, 0.0, 0.0 };
  _prev_error = std::numeric_limits<double>::quiet_NaN();
}

void
PID::UpdateError(double error)
{
  if (std::isnan(_prev_error)) {
    _prev_error = error;
  }

  _errors[0] = error;

  if (!_NeedToClamp(error)) {
    _errors[1] += error;
  } else {
    std::cout << "integrator is shutdown.\n";
  }

  _errors[2] = error - _prev_error;
  _prev_error = error;
}

double
PID::TotalError() const
{
  double total_error = 0.0;
  if (_mode & ControllerMode::kPMode) {
    total_error += _params[0] * _errors[0];
  }
  if (_mode & ControllerMode::kIMode) {
    total_error += _params[1] * _errors[1];
  }
  if (_mode & ControllerMode::kDMode) {
    total_error += _params[2] * _errors[2];
  }
  return total_error;
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

} // end of controller
