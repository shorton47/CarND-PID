//----------
// PID controller for vehicle in Udacity simulator
//
// Using variable naming conventions from Udacity supplied "PID.h"
//----------
#include "PID.h"
#include <iostream>
#include <vector>
#include <numeric>      // std::accumulate

using namespace std;

vector<double>  p  = {0.0, 0.0, 0.0};
vector<double> dp  = {0.0, 0.0, 0.0};


/*
* TODO: Complete the PID class.
*/

// Constructor
PID::PID() {}

// Destructor
PID::~PID() {}

// Init for PID control
void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
    
    Kp = Kp_init;   // Tau_P or Proportional Gain
    Ki = Ki_init;   // Tau_I or Integral Gain
    Kd = Kd_init;   // Tau_D or Differential Gain
    p_error = 0.0;  // This is cross-track error or CTE
    i_error = 0.0;  // This is integral error which is sum of all CTE errors
    d_error = 0.0;  // This is differential error which is d/dt of CTE (This code assumes dt is = 1 !!!)
    
    p[0] = Kp_init;   // Tau_P or Proportional Gain
    p[1] = Ki_init;   // Tau_I or Integral Gain
    p[2] = Kd_init;   // Tau_D or Differential Gain
    
    //p  = {0.0, 0.0, 0.0};
   
    
}

//
// With a new CTE, update all the terms in the PID control loop
//
void PID::UpdateError(double cte) {
    
    d_error = cte - p_error;  // Differential error is d/dt of CTE which= (current CTE - previous CTE. NOTE: dt assumed =1)
    p_error = cte;            // Proportional error is just the new CTE
    i_error += cte;           // Integral error is sum of ALL CTE
    
}

//
// The total error is the Steering Angle ERROR to adjust to get back to center line
//
double PID::TotalError() {
    
    double steer_adjust = -Kp*p_error - Kd*d_error - Ki*i_error;  // Equation from class
    //cout << "cors=" << d_error << " " << p_error << " " << i_error << " " << steer_adjust << endl;
    
    return steer_adjust;

}


//
// The total error is the Steering Angle ERROR to adjust to get back to center line
//
double PID::TotalErrorEstimate() {
    
    double steer_adjust = -p[0]*p_error - p[1]*d_error - p[2]*i_error;  // Equation from class
    //cout << "cors=" << d_error << " " << p_error << " " << i_error << " " << steer_adjust << endl;
    
    return steer_adjust;
    
}

//
// Code from the Forum to reset the Udacity Simulator which allows running a portion of the run during training of PID
// parameters
//
void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


//
// Given current CTE and current PID Errors (p_error, d_error, i_error), find optimal PID coefficients
// that minimize Total CTE. Assume UpdateError has already been called for this point.
// This is a variant of Thrun's TWIDDLE algorithm!
//
void PID::FindOptimalPIDParams() {
    
    
    //p  = {0.0, 0.0, 0.0};
    dp = {1.0, 1.0, 1.0};

    double tol = .001;
    double best_err,err;
    
    //robot = make_robot()
    //p[0] = Kp;
    //p[1] = Ki;
    //p[2] = Kd;
    //KpNew = Kp;
    //KiNew = Ki;
    //KdNew = Kd;
    
    best_err = TotalErrorEstimate();
    cout << "Start error=" << best_err << endl;
    
    int it = 0;
    while (accumulate(dp.begin(),dp.end(),0.0) > tol) {
        
        //print("Iteration {}, best error = {}" + format(it, best_err));
        double sum = accumulate(dp.begin(),dp.end(),0.0);
        cout << it << " " << p[0] << " " << p[1] << " " << p[2] << " " << sum << endl;
        
        for (int i=0; i< dp.size(); i++) {
            p[i] += dp[i];
            err = TotalErrorEstimate();
            
            if (err < best_err) {
                best_err = err;
                dp[i] *= 1.1;
            } else {
                p[i] -= 2.0 * dp[i];
                err = TotalErrorEstimate();
                
                if (err < best_err) {
                    best_err = err;
                    dp[i] *= 1.1;
                } else {
                    p[i] += dp[i];
                    dp[i] *= 0.9;  // Try reducing
                }  // if-else err
            } // if-else err
        
        it += 1;
        } // for each param
        
    } // while error is above tol
    cout << "Converge! p=" << p[0] << " " << p[1] << " " << p[2] << endl;
    
} // FindOptimalPIDParams
