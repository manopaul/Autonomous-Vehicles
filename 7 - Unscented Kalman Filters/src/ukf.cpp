#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;
    
    // initial state vector
    x_ = VectorXd(5);
    
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 3; //30
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.4; //30
    
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;
    
    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
    
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;
    
    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;
    
    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    
    /**
     TODO:
     
     Complete the initialization. See ukf.h for other member properties.
     
     Hint: one or more values initialized above might be wildly off...
     */
    
    //* initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;
    
    // time when the state is true, in us
    time_us_ = 0.0;
    
    // State dimension
    n_x_ = 5;
    
    // Augmented state dimension
    n_aug_ = 7;
    
    // Sigma point spreading parameter
    lambda_ = 3 - n_x_;
    
    // predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    // Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ +1);
    
    // NIS for radar measurements
    NIS_radar_ = 0.0;
    
    // NIS for laser measurements
    NIS_laser_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     Complete this function!
     **/
    
    if((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) ||
       (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)) {
        
        //INTIALIZATION STEP
        
        //Check if initial values are set; if not initialize
        if(!is_initialized_){
            //Initialize state x_ and covariance matrix P_
            
            //1st measurement'
            x_ << 1,1,1,1,0.1;
            
            //Initialize covariance matrix
            P_ <<   0.15, 0,0,0,0,
            0, 0.15,0,0,0,
            0,    0,1,0,0,
            0,    0,0,1,0,
            0,    0,0,0,1;
            
            //set initial timestamp
            time_us_ = meas_package.timestamp_;
            
            if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
                float rho = meas_package.raw_measurements_(0);
                float phi = meas_package.raw_measurements_(1);
                float rho_dot = meas_package.raw_measurements_(2);
                float cosPhi = cos(phi);
                float sinPhi = sin(phi);
                
                x_(0) = rho * cosPhi;
                x_(1) = rho * sinPhi;
            }
            else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
                
                x_(0) = meas_package.raw_measurements_(0);
                x_(1) = meas_package.raw_measurements_(1);
            }
            
            is_initialized_ = true;
            return;
        }
        
        //PREDICTION STEP
        
        float deltat = (meas_package.timestamp_ - time_us_) / 1000000.0; //time delta in seconds
        time_us_ = meas_package.timestamp_;
        
        Prediction(deltat);
        
        //UPDATE STEP
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            UpdateRadar(meas_package);
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            UpdateLidar(meas_package);
        }
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
     Complete this function!
     */
    //Estimate the object's location. Modify the state vector, x_.
    //Predict sigma points, the state, and the state covariance matrix.
    
    /**********************/
    //GENERATE SIGMA POINTS
    /**********************/
    
    MatrixXd Xsig = MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    //sqrt of P
    MatrixXd A = P_.llt().matrixL();
    
    //lambda for generated (non-augmented) sigma points
    lambda_ = 3 - n_x_;
    
    //1st column of sigma points matrix
    Xsig.col(0) = x_;
    
    //Other columns of sigma points matrix set through iteration
    for (int i = 0; i < n_x_; i++) {
        Xsig.col(i+1)        = x_ + sqrt(lambda_ + n_x_) * A.col(i);
        Xsig.col(i+1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
    }
    
    /*********************/
    //AUGMENT SIGMA POINTS
    /*********************/
    
    //Augmented mean
    VectorXd x_aug = VectorXd(n_aug_);
    
    //Augmented state covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    //Create Augmented Sigma Point Matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    
    //Augmented Sigma Lambda
    lambda_ = 3 - n_aug_;
    
    //Create Augmented Mean State
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    //Create Augmented Covariance Matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;
    
    //sqrt Matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    //Create Augmented Sigma Points
    //1st column of augmented sigma points matrix
    Xsig_aug.col(0) = x_aug;
    
    //Other columns of augmented sigma points matrix set through iteration
    for(int i=0; i < n_aug_; i++) {
        Xsig_aug.col(i+1)           = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i+1 + n_aug_)  = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }
    
    /************************/
    //PREDICTING SIGMA POINTS
    /************************/
    
    for(int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        //extracting values for ease of readability
        double p_x       = Xsig_aug(0, i);
        double p_y       = Xsig_aug(1, i);
        double v         = Xsig_aug(2, i);
        double yaw       = Xsig_aug(3, i);
        double yawd      = Xsig_aug(4, i);
        double nu_a      = Xsig_aug(5, i);
        double nu_yawdd  = Xsig_aug(6, i);
        
        //predicted state values
        double px_p, py_p;
        
        //handling division by Zero
        if(fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        }
        else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;
        
        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;
        
        //write predicted sigma points into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
    
    /*******************************/
    //PREDICTING MEAN AND COVARIANCE (Converting Predicted Sigma Points)
    /*******************************/
    
    //Initializing and setting weights
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for(int i = 1; i < 2 * n_aug_ + 1; i++) {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
    
    //Predicted State Mean
    x_.fill(0.0);
    for(int i =0; i < 2 * n_aug_ + 1; i++) { //iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }
    
    //Predicted State Covariance Matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) { //iterate over sigma points
        
        //Finding State Difference (Residual)
        VectorXd xdiff = Xsig_pred_.col(i) - x_;
        
        //Normalizing Angles
        while(xdiff(3) > M_PI)  xdiff(3) -= 2. * M_PI;
        while(xdiff(3) <- M_PI) xdiff(3) += 2. * M_PI;
        
        P_ = P_ + weights_(i) * xdiff * xdiff.transpose();
    }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     Complete this function!
     **/
    // Use lidar data to update the belief about the object's position.
    // Modify the state vector, x_, and covariance, P_.
    // You'll also need to calculate the lidar NIS.
    
    VectorXd z = meas_package.raw_measurements_;
    
    // Measurement space and matrix to hold lidar measurements p_x and p_y
    int n_z = 2;
    
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //Transform Sigma Points to fit Measurement model
    for(int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        //Extracting value for ease of readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        
        // Measuremeant model
        Zsig(0, i) = p_x;
        Zsig(1, i) = p_y;
    }
    
    //Predicted Mean Measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for(int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //Measurement Covariance Matrix 'S'
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        //Finding State Mean Difference (Residual)
        VectorXd zdiff = Zsig.col(i) - z_pred;
        
        S = S + weights_(i) * zdiff * zdiff.transpose();
    }
    
    //Adding Measurement Noise Covariance Matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    
    R << std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;
    
    S = S + R;
    
    
    //Creating the Cross Relatoon Matrix 'Tc'
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    /******************
     UKF Update - LASER
     ******************/
    
    Tc.fill(0.0);
    
    //Calculate Cross correlation matrix
    for(int i =0; i < 2 * n_aug_ + 1; i++) {
        
        //Finding State Difference (Residual)
        VectorXd xdiff = Xsig_pred_.col(i) - x_;
        
        //Finding State Mean Difference (Residual)
        VectorXd zdiff = Zsig.col(i) - z_pred;
        
        Tc = Tc + weights_(i) * xdiff * zdiff.transpose();
    }
    
    //Kalman Gain 'K'
    MatrixXd K = Tc * S.inverse();
    
    //Residual
    VectorXd zdiff = z - z_pred;
    
    //Calculating Accuracy 9NIS)
    NIS_laser_ = zdiff.transpose() * S.inverse() * zdiff;
    
    //Update State and Covariance Matrix
    x_ = x_ + K * zdiff;
    P_ = P_ - K * S * K.transpose();
    
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     Complete this function!
     **/
    //Use radar data to update the belief about the object's position.
    // Modify the state vector, x_, and covariance, P_.
    //You'll also need to calculate the radar NIS.
    
    VectorXd z = meas_package.raw_measurements_;
    
    // Measurement space and matrix to hold rho, phi and rho_dot
    int n_z = 3;
    
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //Transform Sigma Points to fit Measurement model
    for(int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        //Extracting value for ease of readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v   = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        
        double v1  = cos(yaw) * v;
        double v2  = sin(yaw) * v;
        
        // Measuremeant model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);       //rho
        Zsig(1, i) = atan2(p_y, p_x);                   //phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); // rho_dot
        
    }
        //Predicted Mean Measurement
        VectorXd z_pred = VectorXd(n_z);
        z_pred.fill(0.0);
        for(int i = 0; i < 2 * n_aug_ + 1; i++) {
            z_pred = z_pred + weights_(i) * Zsig.col(i);
        }
        
        //Measurement Covariance Matrix 'S'
        MatrixXd S = MatrixXd(n_z, n_z);
        S.fill(0.0);
        for (int i = 0; i < 2 * n_aug_ + 1; i++) {
            
            //Finding State Mean Difference (Residual)
            VectorXd z_diff = Zsig.col(i) - z_pred;
            
            //Normalizing Angles
            while (z_diff(1) > M_PI)  z_diff(1) -= 2. * M_PI;
            while (z_diff(1) <- M_PI) z_diff(1) += 2. * M_PI;
            
            S = S + weights_(i) * z_diff * z_diff.transpose();
        }
        
        //Adding Measurement Noise Covariance Matrix
        MatrixXd R = MatrixXd(n_z, n_z);
        
        R << std_radr_ * std_radr_, 0, 0,
        0, std_radphi_ * std_radphi_, 0,
        0, 0, std_radrd_ * std_radrd_;
        
        S = S + R;
        
        //Creating the Cross Relatoon Matrix 'Tc'
        MatrixXd Tc = MatrixXd(n_x_, n_z);
        
        /******************
         UKF Update - RADAR
         ******************/
        
        Tc.fill(0.0);
        
        //Calculate Cross correlation matrix
        for(int i =0; i < 2 * n_aug_ + 1; i++) {
            
            //Finding State Mean Difference (Residual)
            VectorXd z_diff = Zsig.col(i) - z_pred;
            
            //Normalizing Angles
            while (z_diff(1) > M_PI)  z_diff(1) -= 2. * M_PI;
            while (z_diff(1) <- M_PI) z_diff(1) += 2. * M_PI;

            
            //Finding State Difference (Residual)
            VectorXd x_diff = Xsig_pred_.col(i) - x_;
            
            //Normalizing Angles
            while(x_diff(3) > M_PI)  x_diff(3) -= 2. * M_PI;
            while(x_diff(3) <- M_PI) x_diff(3) += 2. * M_PI;
            
            
            Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
        }
        
        //Kalman Gain 'K'
        MatrixXd K = Tc * S.inverse();
        
        //Residual
        VectorXd z_diff = z - z_pred;
        
        //Normalizing Angles
        while (z_diff(1) > M_PI)  z_diff(1) -= 2. * M_PI;
        while (z_diff(1) <- M_PI) z_diff(1) += 2. * M_PI;
        
        //Calculating Accuracy 9NIS)
        NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
        
        //Update State and Covariance Matrix
        x_ = x_ + K * z_diff;
        P_ = P_ - K * S * K.transpose();
    }
