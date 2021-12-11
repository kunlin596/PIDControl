#include "PID.h"

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
}

void
PID::UpdateError(double cte)
{
  _d_error = cte - _p_error;
  _i_error += cte;
  _p_error = cte;
  /**
   * TODO: Update PID errors based on cte.
   */
}

double
PID::TotalError() const
{
  return _Kp * _p_error + _Ki * _i_error + _Kd * _d_error;
}
