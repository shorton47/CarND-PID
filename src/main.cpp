//----------
// Main module for PID controller Project. This module talks to the Udacity simulator and receives data
// back from the Simulator's vehicle through WebSocket messages. A bulck of this code was starter code from
// Udacity which I modified to handle the results of the "telemetry" message.
//
// Note1: See the main method below for important constansts and current PID tuning parameters.
// Note2: Throttle value currently fixed. Follow-on suggestion to make this in a P loop as well
//----------

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

long train_max_steps_per_iteration, train_max_iterations;
long step,iter;
bool training_mode;
double steer_value;


//-----
// Helper functions
//-----

// hasData checks if the SocketIO event has JSON data. If there is data, the JSON object in string format will be
// returned, else the empty string "" will be returned.
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

// simulatorRestart method restarts the simulator by returning the car back to position zero, speed zero, steering angle
// zero. Using SocketIO to send message.
void restartSimulator(uWS::WebSocket<uWS::SERVER> ws) {
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


//-----
// Main Method. Sets up communication between this module and the Udacity Simulator.
//-----
int main() {

    uWS::Hub h;
    PID pid = PID();
    
    //---
    // PID constants. Depending on mode (train or operational), these are either the default params or they
    // are the tuned PID values.
    //---
    double static P_def_gain = 0.15;    // 0.15   .15    0.1     0.1
    double static I_def_gain = 0.0004;  // 0.0004 .0001  0.0004  0.0001
    double static D_def_gain = 4.0;     // 4.0   4.0     2.0     2.0
    
    training_mode = false;       // To calc optimal PID params. If false, just runs PID loop w/ current saved values
    train_max_iterations = 20;            // Maximum training runs
    train_max_steps_per_iteration = 200;  // Maximum training steps per run
    double throttle = 0.5;  // .5 ~= 50 mph (throttle currently fixed

    
    // Start
    step = 0;
    iter = 0;
    
    if (training_mode) {
        cout << "Main: Training mode." << " Max train steps=" << train_max_iterations << " Steps/iter=" \
             << train_max_steps_per_iteration    << " Throttle=" << throttle << endl;
    } else {
        cout << "Main: Normal operational mode." << " Throttle=" << throttle << endl;
    }
    
    // Init PID w/ default PID parameters
    pid.Init(P_def_gain,I_def_gain,D_def_gain);
    
    
    //
    // Main message handler when message comes in from webSocket
    //
    h.onMessage([&pid,&throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        
        // "42" at the start of the message means there's a websocket message event.
        // 4 signifies a websocket message, 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
            
                    // Update data from car. j[1] is the data JSON object
                    step += 1;
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                   
                    // Training mode or else standard operation mode
                    if (training_mode) {
                        
                        // Process training step
                        pid.UpdateErrors(cte);
                        steer_value = pid.UpdateControl();
                        steer_value = fmin(fmax(steer_value, -1.0), 1.0);  // Clip at 10% over maximum steering value
                        
                        // Reached max steps per iteration
                        if (step == train_max_steps_per_iteration) {
                            pid.TwiddlePIDParams(iter,step);  // pid twiddle needs to be called to update p
                            pid.PrintPIDParams();
                            restartSimulator(ws);
                            step = 0; iter += 1;
                            return;
                        }
                    
                    } else {
                
                        // Regular PID operation w/ tuned values
                        pid.UpdateErrors(cte);
                        steer_value = pid.UpdateControl();
                        steer_value = fmin(fmax(steer_value, -1.0), 1.0);  // Clip to Simulator maximum steering range
                
                    } // if-else train/regular
                    
                    // Pick-up w/ Udacity code
                    cout << "I=" << iter << " Step=" << step << " Speed=" << speed << " Angle=" << angle \
                                 << " CTE="  << cte  << " Adjust=" << steer_value << endl;
                    
                    // Construct message to simulator
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    
                    // Send
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    
                } // if telemetry
                
            } else {
        
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
            } // if-else s blank/not-blank
        
        } // if len
                
    }); //h.onMessage

  
    
  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
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

  // Establish connection between this module & Udacity simulator
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "PID Controller connected & ready to steer car!!!" << std::endl;
  });

  // Dis-connect handler.
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

    
  // Continue on w/ Main
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
    
  // Run message handler
  h.run();
    
} // Main



