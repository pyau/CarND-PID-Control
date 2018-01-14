#ifndef PID_H
#define PID_H
#include <time.h>
#include <ctime>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double bestKp;
  double bestKi;
  double bestKd;
  double dp[3];

  double prev_cte;

  bool first_step;

  double total_error;
  double best_error;
  int curr_index;
  bool increasing[3];
  //clock_t prev_time;
  int step;

  bool twiddle;
  int run;
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
  void Init(double Kp_, double Kd_, double Ki_);

  /*
  * Update the PID error variables given cross track error.
  */
  bool UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void AddToParameter(int ind, double val);
  void ScaleParamUp(int ind);
  void DecreaseParam(int ind);
  void ScaleParamDown(int ind); 

  double CalculateSteer(double speed);
  double CalculateThrottle();
  void EnableTwiddle(bool t);
};

#endif /* PID_H */
