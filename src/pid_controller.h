#ifndef PID_H
#define PID_H

#include <array>
#include <bits/c++config.h>
#include <bits/stdint-uintn.h>
#include <cstdint>
#include <limits>
#include <locale>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace controller {

using Parameters = std::array<double, 3>;

/**
 * @brief      This class describes a coordinate ascent optimizer.
 */
class CoordinateAscentOptimizer
{
public:
  /**
   * @brief      This class describes a optimizer state.
   */
  enum class State : uint8_t
  {
    kInitialized,
    kIncreased,
    kDecreased,
  };

  CoordinateAscentOptimizer(const Parameters param_deltas = { 1.0, 1.0, 1.0 },
                            double increase_scale = 1.1,
                            double decrease_scale = 0.9)
    : _param_deltas(param_deltas)
    , _min_error(std::numeric_limits<double>::infinity())
    , _increase_scale(increase_scale)
    , _decrease_scale(decrease_scale)
  {}

  /**
   * @brief      Collect new errors into optimizer
   *
   * @param[in]  error  The error
   */
  inline void Collect(double error) { _total_error += (error * error); }

  inline void ResetCounter()
  {
    _counter = 0;
    _total_error = 0.0;
  }

  inline bool NeedOptimization() const
  {
    return std::accumulate(_param_deltas.begin(), _param_deltas.end(), 0.0) > 1e-8;
  }

  /**
   * @brief      Optimize the current parameters given a sequence of cached errors
   *
   * @param[in]  parameters  The parameters
   */
  Parameters Optimize(const Parameters& parameters);

  double GetMinError() const { return _min_error; }
  double GetTotalError() const { return _total_error; }

private:
  /**
   * @brief      Sets the next dimension for parameter tweaking.
   */
  inline void _SetNextParam() { _current_dim = (_current_dim + 1) % 3; }

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

  static const int MAX_RECORDS = 10;
  int _counter = 0;
  double _total_error = 0.0;                                   ///< Sum of all collected errors
  Parameters _param_deltas;                                    ///< Parameter deltas to be tuned
  uint32_t _current_dim = 0;                                   ///< Current parameter dimension to be checked
  double _min_error = std::numeric_limits<double>::infinity(); ///< Current minimal error
  double _increase_scale;                                      ///< Increase scale when adding delta succeeded
  double _decrease_scale;                                      ///< Decrease scale when adding delta failed
  State _state = State::kInitialized; ///< Current optimizer state, this state controls what to try next
};

/**
 * @brief      Controller mode to toggling different components in the controller
 */
enum ControllerMode : uint8_t
{
  kPMode = 0x01,
  kIMode = 0x02,
  kDMode = 0x04,
  kPDMode = 0x05,
  kPIDMode = 0x07
};

/**
 * @brief      This class describes a PID controller.
 */
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
  void Init(const Parameters& parameters) { _params = parameters; }

  const Parameters& GetParameters() const { return _params; }

  void SetMode(uint8_t mode) { _mode = mode; }

  /**
   * @brief      Update current error
   *
   * @param[in]  error  The error
   */
  void UpdateError(double error);

  /**
   * @brief      Compute total error for outputting controller signals
   *
   * @return     total error
   */
  double TotalError() const;

private:
  uint8_t _mode = ControllerMode::kPIDMode;                      ///< Control mode
  Parameters _errors = { 0.0, 0.0, 0.0 };                        ///< Errors for PID components
  Parameters _params = { 0.225, 0.0004, 4.0 };                   ///< Parameters for PID components
  double _prev_error = std::numeric_limits<double>::quiet_NaN(); ///< Cached previous error for D component
  double _min = std::numeric_limits<double>::min();              ///< The minimal value of output signal
  double _max = std::numeric_limits<double>::max();              ///< The maximum value of output signal

  /**
   * @brief      Try to clamp the output signal
   *
   * @param[in]  output     The output
   * @param[in]  min_delta  The minimum delta
   * @param[in]  max_delta  The maximum delta
   *
   * @return     clamped output signal
   */
  double _TryClamp(double output, double min_delta, double max_delta) const;

  /**
   * @brief      Check if the output signal is saturated
   *
   * @param[in]  output  The output
   *
   * @return     need to clamp the output signal or not
   */
  bool _NeedToClamp(double output) const;
};
} // end of controller

#endif // PID_H
