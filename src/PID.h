#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

using namespace std;

class PID {
public:
 
    /*
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;
    double sumsq_error;  // SWH add

    /*
     * Coefficients
     */
    double Kp;
    double Ki;
    double Kd;

    
    /*
     * Constructor
     */
    PID();

    /*
     * Destructor.
     */
    virtual ~PID();

    /*
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd);

    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateErrors(double cte);

    /*
     * Calculate the total PID error.
     */
    double UpdateControl();
    
   
    //
    // TWIDDLE Section
    //

    // Run the twiddle (hill-climb) optimizer on PID params
    void TwiddlePIDParams(long iter, long step);

    // Calculate RMSE error on run as figure of merit
    double AccumulatedRMSEError(long n);
    
    // Update PID params during Twiddle
    void UpdatePIDParams(double Kp_cur, double Ki_cur, double Kd_cur);
    
    // Debug to print current PID params
    string PrintPIDParams();

};

#endif /* PID_H */
