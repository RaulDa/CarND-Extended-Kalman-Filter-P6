#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
		     0, 1, 0, 0;

  // measurement matrix - radar
  Hj_ << 0, 0, 0, 0,
		 0, 0, 0, 0,
		 0, 0, 0, 0;

  // state covariance matrix
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 10, 0, 0, 0,
		     0, 10, 0, 0,
			 0, 0, 10000, 0,
			 0, 0, 0, 10000;

  // state transition matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 0, 0,
		     0, 1, 0, 0,
			 0, 0, 1, 0,
			 0, 0, 0, 1;

  // acceleration noise
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    // initialize state vector depending on whether first measurement is radar or laser
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

       // convert radar from polar to cartesian coordinates and initialize state.
       ekf_.x_ << (measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1])), (measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1])), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

       // initialize state.
       ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // define size of process covariance matrix
    ekf_.Q_ = MatrixXd(4, 4);

    // define initial size of R and H with laser matrix sizes
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.H_ = MatrixXd(2, 4);

    // store current timestamp for use in next cycle
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // calculate time difference between measurements and store result
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // do repeated calculations once for updating process covariance matrix
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // add time component to state transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update process covariance matrix depending on time difference and acceleration noise
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
		   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
		   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
		   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  // predict
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/


  // update in function of radar or laser
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

	  // calculate jacobian to be used as measurement matrix
	  Hj_ = tools.CalculateJacobian(ekf_.x_);

	  // resize measurement and measurement covariance matrices for radar
	  ekf_.H_.resize(3, 4);
	  ekf_.R_.resize(3, 3);

	  // set filter matrices
	  ekf_.H_ = Hj_;
	  ekf_.R_ = R_radar_;

	  // do update
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_, tools.CartToPolar(ekf_.x_));
  } else {

	  // resize measurement and measurement covariance matrices for laser
	  ekf_.H_.resize(2, 4);
	  ekf_.R_.resize(2, 2);

	  // set filter matrices
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;

	  // do update
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_.transpose() << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
