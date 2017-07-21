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

vector<double> static  p  = {0.0, 0.0, 0.0};
vector<double> static dp  = {1.0, 1.0, 1.0};


// Constructor
PID::PID() {}

// Destructor
PID::~PID() {}

// Init for PID control
void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
    
    //Kp = Kp_init;   // Tau_P or Proportional Gain
    //Ki = Ki_init;   // Tau_I or Integral Gain
    //Kd = Kd_init;   // Tau_D or Differential Gain
    
    p_error = 0.0;  // This is cross-track error or CTE
    i_error = 0.0;  // This is integral error which is sum of all CTE errors
    d_error = 0.0;  // This is differential error which is d/dt of CTE (This code assumes dt is = 1 !!!)
    sumsq_error = 0.0;  // This is differential error which is d/dt of CTE (This code assumes dt is = 1 !!!)
    
    p[0] = Kp_init;   // Tau_P or Proportional Gain
    p[1] = Ki_init;   // Tau_I or Integral Gain
    p[2] = Kd_init;   // Tau_D or Differential Gain
    
    dp  = {1.0, 1.0, 1.0};
    cout << "PID: controller initilized with: Kp, Ki, Kd =" << p[0] << " " << p[1] << " " << p[2] << endl;
   
    
}

void PID::UpdatePIDParams(double Kp_cur, double Ki_cur, double Kd_cur) {

    
    Kp = Kp_cur;   // Tau_P or Proportional Gain
    Ki = Ki_cur;   // Tau_I or Integral Gain
    Kd = Kd_cur;   // Tau_D or Differential Gain
    
    p[0] = Kp_cur;   // Tau_P or Proportional Gain
    p[1] = Ki_cur;   // Tau_I or Integral Gain
    p[2] = Kd_cur;   // Tau_D or Differential Gain
    

}

string PID::PrintPIDParams() {
    
    return " P=" + to_string(p[0]) + " " + to_string(p[1]) + " " + to_string(p[2]) +
           " DP=" + to_string(dp[0]) + " " + to_string(dp[1]) + " " + to_string(dp[2]) + "\n";
    
    
}



//
// With a new CTE, update all the terms in the PID control loop
//
void PID::UpdateErrors(double cte) {
    
    d_error  = cte - p_error;  // Differential error d/dt of CTE which=(current CTE - previous CTE. NOTE: dt assumed =1)
    p_error  = cte;            // Proportional error is just the new CTE
    i_error += cte;            // Integral error is sum of ALL CTE
    sumsq_error += cte*cte;    // Sum of squared error for RMSE (Error is reference to centerline of 0.0)
    
}

//
// The total error is the Steering Angle ERROR to adjust to get back to center line
//
double PID::UpdateControl() {
    
    //double steer_adjust = -Kp*p_error - Kd*d_error - Ki*i_error;  // Equation from class
    double adjust = -p[0]*p_error - p[1]*i_error - p[2]*d_error;  // Equation from class
    //cout << "PID: errors=" << d_error << " " << p_error << " " << i_error << " " << adjust << endl;
    //cout << "PID: params=" << p[0] << " " << p[1] << " " << p[2] << endl;

    
    return adjust;

}

double PID::AccumulatedRMSEError(long n) {
    
    return sumsq_error/n;

}



//
// The total error is the Steering Angle ERROR to adjust to get back to center line
//
//double PID::TotalErrorTraining(vector<double> p) {
//
//    double steer_adjust = -p[0]*p_error - p[1]*d_error - p[2]*i_error;  // Equation from class
//    //cout << "cors=" << d_error << " " << p_error << " " << i_error << " " << steer_adjust << endl;
//
//    return steer_adjust;
//
//}

//
// Code from the Forum to reset the Udacity Simulator which allows running a portion of the run during training of PID
// parameters
//
//void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
//    std::string reset_msg = "42[\"reset\",{}]";
//    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
//}


//
// Given current CTE and current PID Errors (p_error, d_error, i_error), find optimal PID coefficients
// that minimize Total CTE. Assume UpdateError has already been called for this point.
// This is a variant of Thrun's TWIDDLE algorithm!
//

// Need iter to know when first time
// need step to calculate accumulated error
// This is Sebastian's Twiddle Approach or Hill-Climb
void PID::TwiddlePIDParams(long iter, long total_steps) {
    
    
    //p  = {0.0, 0.0, 0.0};
    //dp = {1.0, 1.0, 1.0};

    //double tol = .001;
    double static best_err = 0.0;
    double static err = 0.0;
    
    //robot = make_robot()
    //p[0] = Kp;
    //p[1] = Ki;
    //p[2] = Kd;
    //KpNew = Kp;
    //KiNew = Ki;
    //KdNew = Kd;
    
    if (iter == 0) {
        best_err = AccumulatedRMSEError(total_steps);
        cout << "Twidde: 1st call. Best RMSE error=" << best_err << endl;
        return;
    }
    
    cout << iter << " Curr  p=" <<  p[0] << " " <<  p[1] << " " <<  p[2] << " " <<  endl;
    cout << iter << " Curr dp=" << dp[0] << " " << dp[1] << " " << dp[2] << " " <<  endl;
    cout << iter << " Curr best error=" << best_err <<  endl;
    
    for (int i=0; i< dp.size(); i++) {
        p[i] += dp[i];
        cout << iter << " new p=" <<  p[0] << " " <<  p[1] << " " <<  p[2] << " " <<  endl;
        err = AccumulatedRMSEError(total_steps);
        cout << i << " error=" << err << " berror=" << best_err <<  endl;
        
        if (err < best_err) {
            cout << i << "!New best error=" << err <<  endl;
            best_err = err;
            dp[i] *= 1.1;
        } else {
            cout << i << " error not lower=" << err <<  endl;
            p[i] -= 2.0 * dp[i];
            cout << i << " else new p=" <<  p[0] << " " <<  p[1] << " " <<  p[2] << " " <<  endl;
            err = AccumulatedRMSEError(total_steps);
            cout << i << " else error=" << err << " berror=" << best_err <<  endl;
            
            if (err < best_err) {
                cout << i << "!New best error=" << err <<  endl;
                best_err = err;
                dp[i] *= 1.1;
            } else {
                p[i] += dp[i];
                cout << i << " else-else new p=" <<  p[0] << " " <<  p[1] << " " <<  p[2] << " " <<  endl;
                dp[i] *= 0.9;  // Try reducing
            }  // if-else err
        } // if-else err
    } // for each param
    
    cout << iter << " New  p=" <<  p[0] << " " <<  p[1] << " " <<  p[2] << " " <<  endl;
    cout << iter << " New dp=" << dp[0] << " " << dp[1] << " " << dp[2] << " " <<  endl;
    //cout << iter << " New best error=" << best_err <<  endl;

    
    
    
} // TwiddlePIDParams
