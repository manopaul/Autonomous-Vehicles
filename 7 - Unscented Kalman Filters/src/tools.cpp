#include <iostream>
#include "tools.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   Calculate the RMSE here.
  **/
    
    VectorXd rmse = VectorXd(4);
    
    //Initialize RMSE vevtor to Zeroes
    rmse << 0,0,0,0;
    
    //Checking validity of input
    //-- estimations vector size should equal ground_truth vector size
    //-- estimations vector size cannot be zero
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Estimations cannot be Zeror or the size of Estimations and Ground Truth vectors dont match." << endl;
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
