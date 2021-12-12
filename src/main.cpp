#include "json.hpp"
#include "pid_controller.h"
#include <cmath>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <iostream>
#include <math.h>
#include <string>
#include <uWS/uWS.h>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
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
constexpr double
pi()
{
  return M_PI;
}

constexpr double
deg2rad(double x)
{
  return x * pi() / 180;
}

double
rad2deg(double x)
{
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string
hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

}

class System
{
public:
  System() { Reset(); }

  void Reset()
  {
    static constexpr double delta_scale = 0.05;
    steering_pid = controller::PID{ 0.225, 0.0004, 4.0, -1.0, 1.0 };
    steering_param_optimizer =
      controller::CoordinateAscentOptimizer{ { steering_pid.GetParameters()[0] * delta_scale,
                                               steering_pid.GetParameters()[1] * delta_scale,
                                               steering_pid.GetParameters()[2] * delta_scale } };

    speed_pid = controller::PID{ 2.25, 0.0001, 1.1, 0.2, 0.8 };
    speed_param_optimizer = controller::CoordinateAscentOptimizer{ { speed_pid.GetParameters()[0] * delta_scale,
                                                                     speed_pid.GetParameters()[1] * delta_scale,
                                                                     speed_pid.GetParameters()[2] * delta_scale } };
  }

  std::string SpinOnce(const json& j)
  {
    // j[1] is the data JSON object
    const json& input_data = j[1];

    double cte = 0.0;
    if (input_data.count("cte")) {
      cte = std::stod(input_data["cte"].get<string>());
    }

    if (std::abs(cte) > 4.0) {
      Reset();
      return "42[\"reset\",{}]";
    }

    double speed = 0.0;
    if (input_data.count("speed")) {
      speed = std::stod(input_data["speed"].get<string>());
    }

    double angle = 0.0;
    if (input_data.count("steering_angle")) {
      angle = deg2rad(std::stod(input_data["steering_angle"].get<string>()));
    }

    double speed_error = std::max((MAX_SPEED - speed) / MAX_SPEED, 0.0);
    double cte_weight = 10.0;
    double speed_weight = 1.0;

    // Update steering PID controller
    double steering_error = cte;
    steering_param_optimizer.Collect(steering_error);
    if (steering_param_optimizer.NeedOptimization()) {
      std::cout << "Steering optimizer activated." << std::endl;
      steering_pid.Init(steering_param_optimizer.Optimize(steering_pid.GetParameters()));
    }

    steering_pid.UpdateError(steering_error);
    double steer_value = (angle - steering_pid.TotalError()) / (1.0 + speed / MAX_SPEED);

    // Update speed PID controller
    double throttle_error = cte_weight * cte + speed_weight * speed_error;
    speed_param_optimizer.Collect(throttle_error);
    if (speed_param_optimizer.NeedOptimization()) {
      std::cout << "Speed optimizer activated." << std::endl;
      speed_pid.Init(speed_param_optimizer.Optimize(speed_pid.GetParameters()));
    }

    speed_pid.UpdateError(throttle_error);
    double throttle_value = std::min(std::max(speed_pid.TotalError() / (1.0 + std::abs(angle) * 10.0), 0.2), 0.8);

    /**
     * TODO: Calculate steering value here, remember the steering value is
     *   [-1, 1].
     * NOTE: Feel free to play around with the throttle and speed.
     *   Maybe use another PID controller to control the speed!
     */

    std::cout << fmt::format("cte: {:7.3f}, speed: {:7.3f}, angle: {:7.3f}", cte, speed, angle) << std::endl;

    std::cout << fmt::format(" - {:>15s}: {:7.3f}, min error: {:10.6f}, total error {:10.6f}, parameters: {}",
                             "steer value",
                             steer_value,
                             steering_param_optimizer.GetMinError(),
                             steering_param_optimizer.GetTotalError(),
                             steering_pid.GetParameters())
              << std::endl;
    std::cout << fmt::format(" - {:>15s}: {:7.3f}, min error: {:10.6f}, total error {:10.6f}, parameters: {}",
                             "throttle value",
                             throttle_value,
                             speed_param_optimizer.GetMinError(),
                             speed_param_optimizer.GetTotalError(),
                             speed_pid.GetParameters())
              << std::endl;

    json msgJson;
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle_value;

    return fmt::format("42[\"steer\",{}]", msgJson.dump());
  }

private:
  controller::PID steering_pid{ 0.225, 0.0004, 4.0, -1.0, 1.0 };
  controller::CoordinateAscentOptimizer steering_param_optimizer;

  controller::PID speed_pid{ 0.025, 0.0015, 0.002, -1.0, 1.0 };
  controller::CoordinateAscentOptimizer speed_param_optimizer;

  static constexpr double MAX_SPEED = 40.0;
  static constexpr double MAX_ANGLE = deg2rad(25.0);
};

int
main()
{
  uWS::Hub h;
  using namespace controller;

  System system;

  h.onMessage([&system](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          std::string msg = system.SpinOnce(j);
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        } // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection(
    [](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
