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
  double sumsq_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

    // SWH Added
    double KpNew;
    double KiNew;
    double KdNew;
  
    
    
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
    
    
    
    double TotalErrorTraining(vector<double> p);
    
    double AccumulatedRMSEError(long n);
    
    void FindOptimalPIDParams();
    
    void Restart(uWS::WebSocket<uWS::SERVER> ws);
    
    void TwiddlePIDParams(long iter, long step);

    string PrintPIDParams();
    
    void UpdatePIDParams(double Kp_cur, double Ki_cur, double Kd_cur);


    
};

#endif /* PID_H */
