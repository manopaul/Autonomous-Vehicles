#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;

  // Initialize the pid variable.
  // P Controller Only
  // pid.Init(0.15,0.0,0.0); //Car Oscillate; reaches max speed of ~35 mph
  // D Controller Only
  // pid.Init(0.0,0.0004,0.0); // Car veers off the road to the left; reaches max speed of ~35 mph
  // I Controller Only
  //pid.Init(0.0,0.0,4.0); // Car veers off the road to the right; reaches max speed of ~40 mph
  // PD Controller Only
  // pid.Init(0.15,0.0004,0.0); // Car oscillates from the start; Veers off road; reaches max speed ~40 mph
  // DI Controller Only
  // pid.Init(0.0,0.0004,4.0); // Car stays on course till a little after the bridge; dont not oscillate as much; veers off road at sharp turn; reaches max speed of ~65 mph
  // PI Controller Only
  // pid.Init(0.15,0.0,4.0); // Interestingly car stayed on course; reaches max speed of ~71 mph; Looks like differential value did not impact
  // PID Controller Only with differential value at 0.5
  // pid.Init(0.15,0.5,4.0); // Interestingly car from the start, started goung in reverse and veers off th road
  // PID Controller
  // pid.Init(0.15,0.0004,3.0); // Car stays on course throughout but wavy on bridge with Ki at 0.3;
  // reaches max speed of ~71 mph
  // PID Controller
  pid.Init(0.15,0.0004,4.0); // Car stays on course throughout; reaches max speed of ~74 mph

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steerValue, throttleValue;

          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steerValue = pid.TotalError();
          throttleValue = (1 - std::abs(steerValue)) * 0.75;
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steerValue << std::endl;
          std::cout << "CTE: " << cte << " Throttle Value: " << throttleValue << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steerValue;
          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttleValue;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
