#ifndef PID_H
#define PID_H

#include <array>
#include <limits>
#include <locale>

class PID
{
public:
  PID() {}
  PID(double Kp, double Ki, double Kd, double min, double max)
    : _params{ Kp, Ki, Kd }
    , _min(min)
    , _max(max)
  {}
  PID(double min, double max)
    : _min(min)
    , _max(max)
  {}

  virtual ~PID() {}

  void Init(double Kp, double Ki, double Kd);

  void UpdateError(double error);

  double TotalError() const;

private:
  std::array<double, 3> _errors = { 0.0, 0.0, 0.0 };
  std::array<double, 3> _params = { 0.225, 0.0004, 4.0 };
  std::array<double, 3> _dparams = { 1.0, 1.0, 1.0 };

  double _prev_error = std::numeric_limits<double>::quiet_NaN();

  /**
   * PID Coefficients
   */
  double _min = std::numeric_limits<double>::min();
  double _max = std::numeric_limits<double>::max();

  double _TryClamp(double output, double min_delta, double max_delta) const;

  bool _NeedToClamp(double output) const;

  /**
   * @brief      Adjust and optimize the parameters using twiddle (officially, coordinate ascent or hill climbing)
   */
  void _Optimize();

  std::array<bool, 6> flags;
};

#endif // PID_H
