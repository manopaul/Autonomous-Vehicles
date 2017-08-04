#include <iostream>
#include "tools.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //Checking validity of input
  //-- estimations vector size should equal ground_truth vector size
  //-- estimations vector size cannot be zero
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Estimations cannot be Zero or the size of Estimations and Ground Truth vectors dont match." << endl;
    return rmse;
  }

  //Accumulate Squared Residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = pow(residual.array(), 2);
    //residual = residual.array() * residual.array();
    rmse += residual;
  }

  //Calculate mean
  rmse = rmse/estimations.size();

  //Calculate Squared Root of mean
  rmse = rmse.array().sqrt();

  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */


  MatrixXd Hj(3,4);
  Hj << 1,1,0,0,
        1,1,0,0,
        1,1,1,1;

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float px2 = px * px;
  float py2 = py * py;
  float px2_and_py2 = px2 + py2; //c1
  float vx_py = vx * py;
  float vy_px = vy * px;
  float sqrt_px2_and_py2 = sqrt(px2_and_py2); //c2
  //float px2_and_py2_times_sqrt_px2_and_py2 = px2_and_py2 * sqrt_px2_and_py2; //c3

  //Check division by Zero
  if(fabs(px2_and_py2) < 0.0001) {
    cout << "Division by Zero encountered" << endl;
    return Hj;
  }

  Hj(0,0) = px / sqrt_px2_and_py2;
  Hj(0,1) = py / sqrt_px2_and_py2;
  Hj(1,0) = -(py / px2_and_py2);
  Hj(1,1) = px / px2_and_py2;
  Hj(2,0) = py * (vx_py - vy_px)/pow(px2_and_py2,1.5);
  Hj(2,1) = px * (vy_px - vx_py)/pow(px2_and_py2, 1.5);
  //Hj(2,0) = py * (vx_py - vy_px)/px2_and_py2_times_sqrt_px2_and_py2;
  //Hj(2,1) = px * (vy_px - vx_py)/px2_and_py2_times_sqrt_px2_and_py2;
  Hj(2,2) = px / sqrt_px2_and_py2;
  Hj(2,3) = py / sqrt_px2_and_py2;

  return Hj;

    /*

    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px + py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    //check division by zero
    if (fabs(c1) < 0.0001){
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
    -(py / c1), (px / c1), 0, 0,
    py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

    return Hj;
    */
}
