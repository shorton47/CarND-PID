//----------
// PID controller for vehicle in Udacity simulator. This class establishes the PID control, tuning paramaters, and also
// an optimizatiom method for tuning.
//
// Note: this uses variable naming conventions supplied from from Udacity in "PID.h"
//----------
#include "PID.h"
#include <iostream>
#include <vector>
#include <numeric>      // std::accumulate

using namespace std;

vector<double> static  p  = {0.0, 0.0, 0.0};  // Store the PID loop tuning parameters
vector<double> static dp  = {1.0, 1.0, 1.0};  // Store the PID loop delta p tuning parameters


// Constructor
PID::PID() {}

// Destructor
PID::~PID() {}

//
// Init for PID control
//
void PID::Init(double Kp_init, double Ki_init, double Kd_init) {
    
    p[0] = Kp_init;   // Tau_P or Proportional Gain
    p[1] = Ki_init;   // Tau_I or Integral Gain
    p[2] = Kd_init;   // Tau_D or Differential Gain
    
    dp  = {1.0, 1.0, 1.0};  // Start with the deltas at 1.0 before tuning
    
    
    p_error = 0.0;  // This is cross-track error or CTE
    i_error = 0.0;  // This is integral error which is sum of all CTE errors
    d_error = 0.0;  // This is differential error which is d/dt of CTE (This code assumes dt is = 1 !!!)
    sumsq_error = 0.0;  // This is differential error which is d/dt of CTE (This code assumes dt is = 1 !!!)
    
    
    cout << "PID: controller initilized with: Kp, Ki, Kd =" << p[0] << " " << p[1] << " " << p[2] << endl;
}

//
// With a new CTE from vehicle, update all the terms in the PID control loop
// Note: calc order is important
//
void PID::UpdateErrors(double cte) {
    
    d_error  = cte - p_error;  // Differential error d/dt of CTE which=(current CTE - previous CTE. NOTE: dt assumed =1)
    p_error  = cte;            // Proportional error is just the new CTE
    i_error += cte;            // Integral error is sum of ALL CTE
    sumsq_error += cte*cte;    // Sum of squared error for RMSE (Error is reference to centerline of 0.0)
}

//
// The PID output (or adjust, or error) is the PID euwquation referenced to the center line of 0.0
//
double PID::UpdateControl() {
    
    double adjust = -p[0]*p_error - p[1]*i_error - p[2]*d_error;  // Equation from class
    //cout << "PID: errors=" << p_error << " " << i_error << " " << d_error << " " << adjust << endl;
    //cout << "PID: params=" << p[0] << " " << p[1] << " " << p[2] << endl;
    return adjust;
}


//----------
// Helper methods
//----------
void PID::UpdatePIDParams(double Kp_cur, double Ki_cur, double Kd_cur) {
    
    p[0] = Kp_cur;   // Tau_P or Proportional Gain
    p[1] = Ki_cur;   // Tau_I or Integral Gain
    p[2] = Kd_cur;   // Tau_D or Differential Gain
}

string PID::PrintPIDParams() {
    
    return " P=" + to_string(p[0]) + " " + to_string(p[1]) + " " + to_string(p[2]) +
           " DP=" + to_string(dp[0]) + " " + to_string(dp[1]) + " " + to_string(dp[2]) + "\n";
}


//----------
// Find the optimal PID coefficients that minimize Total CTE. This is a variant of Thrun's TWIDDLE (or Hill-Climb)
// algorithm in an interrupt driven environment! This uses the accumulated RMSE from a "run" to establish the error for a
// given set of PID parameters to then decide how to modify for next run.
//----------
void PID::TwiddlePIDParams(long iter, long total_steps) {

    double static best_err = 0.0;
    double static err = 0.0;

    
    // Default RMSE is from first call
    if (iter == 0) {
        best_err = AccumulatedRMSEError(total_steps);
        cout << "Twidde: 1st call. Best RMSE error=" << best_err << endl;
        return;
    }
    
    // Debug
    cout << iter << " Curr  p=" <<  p[0] << " " <<  p[1] << " " <<  p[2] << " " <<  endl;
    cout << iter << " Curr dp=" << dp[0] << " " << dp[1] << " " << dp[2] << " " <<  endl;
    cout << iter << " Curr best error=" << best_err <<  endl;
    
    // Twiddle algorithm
    for (int i=0; i< dp.size(); i++) {
        p[i] += dp[i];
        err = AccumulatedRMSEError(total_steps);
        
        if (err < best_err) {
            best_err = err;
            dp[i] *= 1.1;
        } else {
            p[i] -= 2.0 * dp[i];
            err = AccumulatedRMSEError(total_steps);
            
            if (err < best_err) {
                best_err = err;
                dp[i] *= 1.1;
            } else {
                p[i] += dp[i];
                dp[i] *= 0.9;  // Try reducing
            }  // if-else err
        } // if-else err
    } // for each param
    
    // Debug
    cout << iter << " New  p=" <<  p[0] << " " <<  p[1] << " " <<  p[2] << " " <<  endl;
    cout << iter << " New dp=" << dp[0] << " " << dp[1] << " " << dp[2] << " " <<  endl;
  
} // TwiddlePIDParams


// Calculate RMSE from the entire run
double PID::AccumulatedRMSEError(long n) {
    
    return sumsq_error/(double)n;
    
}
