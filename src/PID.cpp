#include <limits>
#include "PID.h"
#include <cmath>
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    bestKp = 0.0;
    bestKi = 0.0;
    bestKd = 0.0;
    dp[0] = 0.2*Kp_;
    dp[1] = 0.2*Kd_;
    dp[2] = 0.2*Ki_;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    prev_cte = 0.0;

    first_step = true;
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    curr_index = 0;
    increasing[0] = increasing[1] = increasing[2] = true;

    time(&prev_time);
    step = 0;
    twiddle = false;    // set to true for twiddle
    run = 0;
}

double PID::CalculateSteer(double speed) {
    double steer = -Kp   *p_error - Kd  *d_error - Ki *i_error;
    if (steer > 1)
        steer = 1;
    else if (steer < -1)
        steer = -1;
    return steer;
}

bool PID::UpdateError(double cte) {

    if (first_step) {
        first_step = false;
        p_error = cte;
        prev_cte = cte;
        d_error = 0.0;
    } else {
        p_error = cte;
        d_error = (cte - prev_cte);
        prev_cte = cte;
    }
    i_error += cte;

    step++;

    // twiddle

    int step_settle = 200;//150;    // settle step
    int step_run = 2000;//500;      // how many steps simulation should run
    int step_tot = step_settle + step_run;

    if ( twiddle == true ) {
        if (step  > step_settle)
            total_error += cte * cte;

        if (step == step_tot)
        {   
            run++;
            cout << "run:" << run << " current error: " << total_error << " best error: " << best_error << endl;
            if (total_error < best_error) {
                best_error = total_error;
                bestKp = Kp;
                bestKd = Kd;
                bestKi = Ki;
                dp[curr_index] *= 1.1;
                if (increasing[curr_index]) {
                    AddToParameter(curr_index, dp[curr_index]);
                } else {
                    AddToParameter(curr_index, -dp[curr_index]);
                }
            } else {
                if (increasing[curr_index]) {
                    // try decreasing
                    AddToParameter(curr_index, -2.0*dp[curr_index]);
                    increasing[curr_index] = false;
                } else {
                    // back to increase, lower dp rate
                    AddToParameter(curr_index, dp[curr_index]);
                    dp[curr_index] *= 0.9;
                    increasing[curr_index] = true;
                    curr_index = (curr_index+1) % 3;
                    AddToParameter(curr_index, dp[curr_index]);
                    //increasing[curr_index] = true;
                }
            }
            cout << "New run K:" << Kp << " " << Kd << " " << Ki  << " dp:" << dp[0] << " " << dp[1] << " " << dp[2]
                << " best: " << bestKp << " " << bestKd<< " " << bestKi <<endl;
        }
    }


    if (twiddle == true && step == step_tot) {
        total_error = 0.0;
        step = 0;
        p_error = 0.0;
        i_error = 0.0;
        d_error = 0.0;
        prev_cte = 0.0;

        first_step = true;
        return true;
    }
    else
        return false;
}

void PID::AddToParameter(int ind, double val) {
    if (ind == 0) {
        Kp += val;
    } else if (ind == 1) {
        Kd += val;
    } else {
        Ki += val;
    }
}

double PID::TotalError() {
    return -1;    
}

