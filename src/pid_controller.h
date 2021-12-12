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

/**
 * @brief      This class describes a coordinate ascent optimizer.
 *
 * @tparam     NumParams  Number of parameters, in this project is always 3.
 */
template<uint32_t NumParams = 3>
class CoordinateAscentOptimizer
{
public:
  /**
   * @brief      This class describes a optimizer state.
   */
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
    std::fill(_param_deltas.begin(), _param_deltas.end(), 0.001);
  }

  /**
   * @brief      Collect new errors into optimizer
   *
   * @param[in]  error  The error
   */
  inline void Collect(double error) { _errors.push_back(error * error); }

  inline bool NeedOptimization() const
  {
    return std::accumulate(_param_deltas.begin(), _param_deltas.end(), 0.0) > 1e-6;
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
        optimized[_current_dim] += _param_deltas[_current_dim];
        _state = State::Increased;
        break;
      case State::Increased:
        // Increased parameters in initialization step.
        // Check the error of increased parameters to decide what to do next
        if (latest_error < _min_error) {
          // Update latest error
          _min_error = latest_error;
          // Increase the parameter delta
          _ScaleDelta(_param_deltas[_current_dim], _increase_scale);
          // Succeeded in current dimension, rotate to next dimension.
          _SetNextDim();
          // The current parameter increment succeeded, increase the next dim
          _ModifyParam(optimized[_current_dim], _param_deltas[_current_dim]);
          // Keep increase state for the next parameter
          _state = State::Increased;
        } else {
          // The previous increment of parameter failed, move to the opposite direction
          _ModifyParam(optimized[_current_dim], -2 * _param_deltas[_current_dim]);
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
          _ScaleDelta(_param_deltas[_current_dim], _increase_scale);
          // Succeeded in current dimension, rotate to next dimension.
          _SetNextDim();
          // The current parameter increment succeeded, increase the next dim
          _ModifyParam(optimized[_current_dim], _param_deltas[_current_dim]);
          // Keep increase state for the next parameter.
          _state = State::Increased;
        } else {
          // Decrement of the current parameter doesn't work, restore the original parameter value.
          _ModifyParam(optimized[_current_dim], _param_deltas[_current_dim]);
          // Decrease the parameter delta a bit since it failed, we need to be a bit more conservative.
          _ScaleDelta(_param_deltas[_current_dim], _decrease_scale);
          // Succeeded in current dimension, rotate to next dimension.
          _SetNextDim();
          // The current parameter increment succeeded, increase the next dim.
          _ModifyParam(optimized[_current_dim], _param_deltas[_current_dim]);
          // Keep increase state for the next parameter.
          _state = State::Increased;
        }
        break;
    }
    return optimized;
  }

private:
  /**
   * @brief      Sets the next dimension for parameter tweaking.
   */
  inline void _SetNextDim() { _current_dim = (_current_dim + 1) % NumParams; }

  /**
   * @brief      Scale the parameter delta given the tweak succeeded or not
   *
   * @param      delta  The parameter delta
   * @param[in]  scale  The scale
   */
  static inline void _ScaleDelta(double& delta, double scale = 1.0) { delta *= scale; }

  /**
   * @brief      Modify the parameter with delta
   *
   * @param      param  The parameter
   * @param[in]  delta  The delta
   */
  static inline void _ModifyParam(double& param, double delta) { param += delta; }

  std::vector<double> _errors;                                 ///< Collected error records
  std::array<double, NumParams> _param_deltas;                 ///< Parameter deltas to be tuned
  uint32_t _current_dim = 0;                                   ///< Current parameter dimension to be checked
  double _min_error = std::numeric_limits<double>::infinity(); ///< Current minimal error
  double _increase_scale;                                      ///< Increase scale when adding delta succeeded
  double _decrease_scale;                                      ///< Decrease scale when adding delta failed
  State _state = State::Initialized; ///< Current optimizer state, this state controls what to try next
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
