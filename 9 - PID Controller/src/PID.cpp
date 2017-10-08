#include "PID.h"
#include <iostream>
#include <limits>
#include <numeric>
#include <math.h>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp, double Ki, double Kd) {
    // Proportional Coefficient
    this->Kp = Kp; //tau_p
    // Integral Coefficient
    this->Ki = Ki; //tau_i
    // Differential Coefficient
    this->Kd = Kd; //tau_d

    //Initialize All Errors to Zero
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  // steering angle = - tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
  return -1.0 * (Kp * p_error + Kd * d_error + Ki * i_error);
}
