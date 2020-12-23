#include "iostream"
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

#define PI 3.14159265


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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
   * TODO: predict the state
   */
    
    

  //Use the state using the state transition matrix
  x_ = F_ * x_;
  //Update the covariance matrix using the process noise and state transition matrix
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;

  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();

  //Update State
  x_ = x_ + (K * y);
  //Update covariance matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);  
  P_ = (I - K*H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);
  
  //Convert the predictions into polar coordinates
    double rho_p = sqrt(px*px + py*py);
    double theta_p = atan2(py,px);

    if (rho_p < 0.0001) {
        cout << "Small prediction value - reassigning Rho_p to 0.0001 to avoid division by zero";
        rho_p = 0.0001;
    }

    double rho_dot_p = (px * vx + py * vy) / rho_p;

    VectorXd z_pred = VectorXd(3);
    z_pred << rho_p, theta_p, rho_dot_p;

    VectorXd y = z - z_pred;

    //Adjust the value of theta if it is outside of [-PI, PI]
    if (y(1) > PI) {
        y(1) = y(1) - 2 * PI;
    }

    else if (y(1) < -PI) {
        y(1) = y(1) + 2 * PI;
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;

    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();

    //Update State
    x_ = x_ + (K * y);
    //Update covariance matrix
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

}
