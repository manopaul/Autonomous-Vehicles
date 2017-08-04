#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/* Not used to check for division by Zero
//Used fabs method instead
class DivideByZeroException {
public:
  DivideByZeroException()
  : message("Division by Zero Error") {}
  const char *errorMsg() const { return message; }
private:
  const char *message;
};

float getQuotient(float numerator, float denominator) {
  if(denominator == 0)
    throw DivideByZeroException();
    return static_cast<float>(numerator)/denominator;
}
*/

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = (F_ * x_);
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::UpdateEKFforLaser(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd predicted_z = H_ * x_;
  //UpdateWithNewEstimate(z, predicted_z);
    VectorXd y = z - predicted_z;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    
    //new estimate (state)
    x_ = x_ + (K * y);
    long sizeOfx_ = x_.size();
    MatrixXd I = MatrixXd::Identity(sizeOfx_, sizeOfx_);
    P_ = (I - (K * H_)) * P_;

}

void KalmanFilter::UpdateEKFforRADAR(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float rho = sqrt( x_(0) * x_(0) + x_(1) * x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;
  //Checking for division by 0
  if(fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0) * x_(2) + x_(1) * x_(3))/rho;
  }
  VectorXd predicted_z(3);
  predicted_z << rho, phi, rho_dot;
  //UpdateWithNewEstimate(z, predicted_z);
    VectorXd y = z - predicted_z;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;
    
    //new estimate (state)
    x_ = x_ + (K * y);
    long sizeOfx_ = x_.size();
    MatrixXd I = MatrixXd::Identity(sizeOfx_, sizeOfx_);
    P_ = (I - (K * H_)) * P_;

}

void KalmanFilter::UpdateWithNewEstimate(const VectorXd &z, VectorXd &predicted_z) {
  VectorXd y = z - predicted_z;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new estimate (state)
  x_ = x_ + (K * y);
  long sizeOfx_ = x_.size();
  MatrixXd I = MatrixXd::Identity(sizeOfx_, sizeOfx_);
  P_ = (I - (K * H_)) * P_;
}
