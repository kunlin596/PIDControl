#include "pid_controller.h"
#include "json.hpp"
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
constexpr double
pi()
{
  return M_PI;
}

double
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
  std::string SpinOnce(const json& j)
  {
    // j[1] is the data JSON object
    const json& input_data = j[1];

    double cte = 0.0;
    if (input_data.count("cte")) {
      cte = std::stod(input_data["cte"].get<string>());
    }

    if (std::abs(cte) > 4.0) {
      return "42[\"reset\",{}]";
    }

    double speed = 0.0;
    if (input_data.count("speed")) {
      speed = std::stod(input_data["speed"].get<string>());
    }

    double angle = 0.0;
    if (input_data.count("angle")) {
      angle = deg2rad(std::stod(input_data["steering_angle"].get<string>()));
    }

    steering_param_optimizer.Collect(cte);
    if (steering_param_optimizer.NeedOptimization()) {
      std::cout << "Steering optimizer activated." << std::endl;
      steering_pid.Init(steering_param_optimizer.Optimize(steering_pid.GetParameters()));
    }

    steering_pid.UpdateError(cte);
    double steer_value = angle - steering_pid.TotalError();

    speed_param_optimizer.Collect(cte);
    if (speed_param_optimizer.NeedOptimization()) {
      std::cout << "Speed optimizer activated." << std::endl;
      speed_pid.Init(speed_param_optimizer.Optimize(speed_pid.GetParameters()));
    }

    speed_pid.UpdateError(cte);
    double speed_value = speed - speed_pid.TotalError();

    /**
     * TODO: Calculate steering value here, remember the steering value is
     *   [-1, 1].
     * NOTE: Feel free to play around with the throttle and speed.
     *   Maybe use another PID controller to control the speed!
     */

    // DEBUG
    std::cout << fmt::format("CTE: {:7.3f}, steering value: {:7.3f}, parameters: {}, speed: {:7.3f}, parameters={}",
                             cte,
                             steer_value,
                             steering_pid.GetParameters(),
                             speed_value,
                             speed_pid.GetParameters())
              << std::endl;

    json msgJson;
    // msgJson["speed"] = speed_value;
    msgJson["steering_angle"] = speed_pid.TotalError();
    msgJson["throttle"] = 0.3;

    return fmt::format("42[\"steer\",{}]", msgJson.dump());
  }

private:
  controller::PID steering_pid{ 0.225, 0.0004, 4.0, -1.0, 1.0 };
  controller::CoordinateAscentOptimizer<3> steering_param_optimizer;

  controller::PID speed_pid{ 1.0, 1.0, 1.0, 0.0, 20.0 };
  controller::CoordinateAscentOptimizer<3> speed_param_optimizer;
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
    [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
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
