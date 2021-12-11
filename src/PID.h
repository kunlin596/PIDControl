#ifndef PID_H
#define PID_H

#include <limits>

class PID
{
public:
  PID() {}
  PID(double Kp, double Ki, double Kd, double min, double max)
    : _Kp(Kp)
    , _Ki(Ki)
    , _Kd(Kd)
    , _min(min)
    , _max(max)
  {}

  virtual ~PID() {}

  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param error The current cross track error
   */
  void UpdateError(double error);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError() const;

private:
  /**
   * PID Errors
   */
  double _p_error = 0.0;
  double _i_error = 0.0;
  double _d_error = 0.0;
  double _prev_error = 0.0;

  /**
   * PID Coefficients
   */
  double _Kp = 100.0;
  double _Ki = 100.0;
  double _Kd = 100.0;
  double _min = std::numeric_limits<double>::min();
  double _max = std::numeric_limits<double>::max();

  double _TryClamp(double output, double min_delta, double max_delta) const;

  bool _NeedToClamp(double output) const;
};

#endif // PID_H
