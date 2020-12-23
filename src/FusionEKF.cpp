#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  
  cout << "1.EKFInit(constructor)\n";
    
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // Initialize state transition matrix
  ekf_.F_ = MatrixXd(4, 4);

  // Initialize state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);

  // Initialize process covariacne matrix
  ekf_.Q_ = MatrixXd(4, 4);

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // set the values for acceleration noise
  noise_ax = 9;
  noise_ay = 9;



}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "2.EKFInit(Firstmaesure)\n";
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

        double rho = measurement_pack.raw_measurements_[0];
        double theta = measurement_pack.raw_measurements_[1];
        double rho_dot = measurement_pack.raw_measurements_[2];
       
        ekf_.x_(0) = rho * cos(theta);
        ekf_.x_(1) = rho * sin(theta);

        cout << measurement_pack.raw_measurements_<<"\n";

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.

        double px = measurement_pack.raw_measurements_[0];
        double py = measurement_pack.raw_measurements_[1];

        ekf_.x_(0) = px;
        ekf_.x_(1) = py;
        cout << measurement_pack.raw_measurements_<<"\n";

    }
    //Capture the timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    //assign initial values to the state transition matrix, F
    ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
    //assign initial values to the covariance matrix, P. Adjust the variance values to reflect uncertainty in initial state
    ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 500, 0,
        0, 0, 0, 500;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  //Update the state transition matrix, F
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  //Set the process covariance matrix, Q
  ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
                0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
             dt3 / 2 * noise_ax, 0, dt2* noise_ax, 0,
                0, dt3 / 2 * noise_ay, 0, dt2* noise_ay;
  
  
  cout << "3.EKF(Predict)\n";
  ekf_.Predict();
  cout << "x_ = " << ekf_.x_[0] << "\t" << ekf_.x_[1] << "\t" << ekf_.x_[2] << "\t" << ekf_.x_[3] << "\t" << endl;
  cout << "P_ = " << ekf_.P_(0, 0) << "\t" << ekf_.P_(1, 1) << "\t" << ekf_.P_(2, 2) << "\t" << ekf_.P_(3, 3) << "\t" << endl;

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  cout << "4.EKF(Update)\n";
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    //Calculate the Jacobian matrix about the current predicted state and set the EKF state transition matrix, H
    //Tools Jacobian;
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;

    //Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
      ekf_.R_ = MatrixXd(3, 3);
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_); //Comment this line to turn off radar updates   

  } else {
    // TODO: Laser updates
      ekf_.H_ = H_laser_;
      //Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
      ekf_.R_ = MatrixXd(2, 2);
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_); //Comment this line to turn off LIDAR updates   
  }

  // print the output
  cout << "x_ = " << ekf_.x_[0] <<"\t"<< ekf_.x_[1] << "\t" << ekf_.x_[2] << "\t" << ekf_.x_[3] << "\t" << endl;
  cout << "P_ = " << ekf_.P_(0,0) << "\t" << ekf_.P_(1, 1) << "\t" << ekf_.P_(2, 2) << "\t" << ekf_.P_(3, 3) << "\t" << endl;
}
