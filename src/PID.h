#ifndef PID_H
#define PID_H

class PID
{
public:
  PID() {}
  PID(double Kp, double Ki, double Kd)
    : _Kp(Kp)
    , _Ki(Ki)
    , _Kd(Kd)
  {}

  virtual ~PID() {}

  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

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

  /**
   * PID Coefficients
   */
  double _Kp = 0.0;
  double _Ki = 0.0;
  double _Kd = 0.0;
};

#endif // PID_H
