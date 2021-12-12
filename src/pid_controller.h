#ifndef PID_H
#define PID_H

#include <array>
#include <bits/c++config.h>
#include <bits/stdint-uintn.h>
#include <cstdint>
#include <limits>
#include <locale>
#include <numeric>
#include <vector>

namespace controller {

template<uint32_t NumParams>
class CoordinateAscentOptimizer
{
public:
  enum class State : uint32_t
  {
    Initialized = 0,
    Increased = 1,
    Decreased = 2
  };

  CoordinateAscentOptimizer(double increase_scale = 1.1, double decrease_scale = 0.9)
    : _increase_scale(increase_scale)
    , _decrease_scale(decrease_scale)
  {
    std::fill(_delta_params.begin(), _delta_params.end(), 0.001);
  }

  /**
   * @brief      Collect new errors into optimizer
   *
   * @param[in]  error  The error
   */
  inline void Collect(double error) { _errors.push_back(error * error); }

  inline bool NeedOptimization() const
  {
    return std::accumulate(_delta_params.begin(), _delta_params.end(), 0.0) > 1e-6;
  }

  /**
   * @brief      Optimize the current parameters given a sequence of cached errors
   *
   * @param[in]  parameters  The parameters
   */
  std::array<double, NumParams> Optimize(const std::array<double, NumParams>& parameters)
  {
    std::array<double, NumParams> optimized = parameters;
    double latest_error = _errors[_errors.size() - 1];
    switch (_state) {
      case State::Initialized:
        // Increase the parameters in the given dimension
        _min_error = latest_error;
        optimized[_current_dim] += _delta_params[_current_dim];
        _state = State::Increased;
        break;
      case State::Increased:
        // Increased parameters in initialization step.
        // Check the error of increased parameters to decide what to do next
        if (latest_error < _min_error) {
          // Update latest error
          _min_error = latest_error;
          // Increase the parameter delta
          _ScaleDelta(_delta_params[_current_dim], _increase_scale);
          // Succeeded in current dimension, rotate to next dimension.
          _SetNextDim();
          // The current parameter increment succeeded, increase the next dim
          _ModifyParam(optimized[_current_dim], _delta_params[_current_dim]);
          // Keep increase state for the next parameter
          _state = State::Increased;
        } else {
          // The previous increment of parameter failed, move to the opposite direction
          _ModifyParam(optimized[_current_dim], -2 * _delta_params[_current_dim]);
          // Clamp the parameter to be bigger than 0
          if (optimized[_current_dim] < 0.0) {
            optimized[_current_dim] = 0.0;
            // Because this parameter is set to 0.0, the effect of the error term will be eliminated from the total
            // error, check next sim.
            _SetNextDim();
          }
          _state = State::Decreased;
        }
        break;
      case State::Decreased:
        // Decreased parameters in the increased step, because it the error check failed.
        // Check the error of decreased parameters to decide what to do next,
        if (latest_error < _min_error) {
          // Update latest error
          _min_error = latest_error;
          // Increase the parameter delta a bit since it succeeded, we can be a bit more aggressive.
          _ScaleDelta(_delta_params[_current_dim], _increase_scale);
          // Succeeded in current dimension, rotate to next dimension.
          _SetNextDim();
          // The current parameter increment succeeded, increase the next dim
          _ModifyParam(optimized[_current_dim], _delta_params[_current_dim]);
          // Keep increase state for the next parameter.
          _state = State::Increased;
        } else {
          // Decrement of the current parameter doesn't work, restore the original parameter value.
          _ModifyParam(optimized[_current_dim], _delta_params[_current_dim]);
          // Decrease the parameter delta a bit since it failed, we need to be a bit more conservative.
          _ScaleDelta(_delta_params[_current_dim], _decrease_scale);
          // Succeeded in current dimension, rotate to next dimension.
          _SetNextDim();
          // The current parameter increment succeeded, increase the next dim.
          _ModifyParam(optimized[_current_dim], _delta_params[_current_dim]);
          // Keep increase state for the next parameter.
          _state = State::Increased;
        }
        break;
    }
    return optimized;
  }

private:
  inline void _SetNextDim() { _current_dim = (_current_dim + 1) % NumParams; }
  inline void _ScaleDelta(double& delta, double scale = 1.0) const { delta *= scale; }
  inline void _ModifyParam(double& param, double delta) const { param += delta; }

  std::vector<double> _errors;
  std::array<double, NumParams> _delta_params;
  uint32_t _current_dim = 0;
  double _min_error = std::numeric_limits<double>::infinity();
  State _state = State::Initialized;
  double _increase_scale;
  double _decrease_scale;
};

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
  void Init(const std::array<double, 3>& parameters) { _params = parameters; }

  const std::array<double, 3>& GetParameters() const { return _params; }

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
};
} // end of controller

#endif // PID_H
