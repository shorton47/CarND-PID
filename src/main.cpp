#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

long n = 0;
bool training_mode = false;
double steer_value;

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

int main() {

    uWS::Hub h;

    PID pid;
  
    // Initial PID variables from manual tune per write-up
    double P_gain = 0.15;    // .15    .1    .1
    double I_gain = 0.0004;  //  .0001  .0004 .0001
    double D_gain = 4.0;     //   4.0  2.0    2.0
    pid.Init(P_gain,I_gain,D_gain);

  //  bool first_call = TRUE;
  //  double diff_cte=0.0,int_cte=0.0;
   
    
    
    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
            
            // j[1] is the data JSON object
            n += 1;
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());
            cout << "speed=" << speed << " angle=" << angle << " cte=" << cte << endl;
        
            //
            // TODO: Calcuate steering value here, remember the steering value is [-1, 1].
            // NOTE: Feel free to play around with the throttle and speed. Maybe use
            // another PID controller to control the speed!
            //
            if (training_mode) {
                
                pid.UpdateError(cte);
                pid.FindOptimalPIDParams();
                steer_value = pid.TotalErrorEstimate();
                steer_value = fmin(fmax(steer_value, -1.1), 1.1);  // Clip at 10% over maximum steering value

            } else {
            
                pid.UpdateError(cte);
                steer_value = pid.TotalError();
                steer_value = fmin(fmax(steer_value, -1.0), 1.0);  // Clip at 10% over maximum steering value
            
            }
            // DEBUG
            //cout << "CTE=" << cte << " Steering Value=" << steer_value << std::endl;

            // Construct message to simulator
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;  // Max throttle ??
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;  // Debug
          
            // Send
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
    std::cout << "PID Controller connected & ready to steer car!!!" << std::endl;
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
