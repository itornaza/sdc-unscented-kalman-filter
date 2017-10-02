#include "ukf.h"
#include <iostream>
#include "Eigen/Dense"
#include "constants.h"

using namespace std;
using namespace Constants;
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
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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
  
  // State dimension
  int n_x_ = 5;
  
  // Augmented state dimension
  int n_aug_ = 7;
  
  // Sigma point spreading parameter
  double lambda_ = 3 - n_aug_;
  
  // Set vector for weights
  VectorXd weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int ix = 1; ix < (2 * n_aug_ + 1); ix++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(ix) = weight;
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::Initialize(MeasurementPackage meas_package) {
  // The 5 state vector's elements for the CTRV model
  // x = [px, py, v, ψ, ψ']
  float px = 0;
  float py = 0;
  float v = 0;
  float psi = 0;
  float psi_d = 0;
  
  // Initialize the filter depending on the sensor that sent the
  // measurement (radar or lidar)
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR &&
      use_radar_) {
    // Get the radar measurements from the pack: [ρ, φ, ρ']
    float rho = meas_package.raw_measurements_[0];      // ρ
    float phi = meas_package.raw_measurements_[1];      // φ
    float rho_dot = meas_package.raw_measurements_[2];  // ρ'
    
    // Convert the position from polar to cartesian coordinates
    // Discard the velocity because it is the radial velocity which
    // differs from the object velocity
    // Note: radar radial velocity (ρ') differs from the object's velocity
    // v, so we set it to zero
    px = rho * cos(phi);
    py = rho * sin(phi);
    
    // Initialize only on the very first measurement
    if (DEBUG) { cout << "Radar initialization measurement" << endl; }
    
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER &&
             use_laser_) {
    // Get the location coordinates from the pack
    // Note: lidar does not provide velocity measurements at all
    float px = meas_package.raw_measurements_[0];
    float py = meas_package.raw_measurements_[1];
    
    // Initialize only on the very first measurement
    if (DEBUG) { cout << "Lidar initialization measurement" << endl; }
  } else {
    if (DEBUG) { cout << "All sensors are off" << endl; }
  }
  
  // Avoid the px = 0 and py = 0 case for initialization
  if (fabs(px) < E1 && fabs(py) < E1) {
    if (DEBUG) { cout << "Initialization with px = py = 0" << endl; }
    px = E1;
    py = E1;
  }
  
  // Initialize the kalman filter with the current position and zero velocity
  x_ << px, py, v , psi, psi_d;
  is_initialized_ = true;
  
  // Update the previous timestamp with the initial measurement timestamp
  time_us_ = meas_package.timestamp_;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // Distinguish between a first or a subsequent measurement
  if (!is_initialized_) {
    //-----------------
    // Initialization
    //-----------------
    Initialize(meas_package);
    
  } else {
    // Compute the time elapsed between the current and previous measurements
    // dt is expressed in seconds where the pack provides msecs
    float dt = (meas_package.timestamp_ - time_us_) / MSEC_TO_SEC;
    
    // Update the previous timestamp with the measurement timestamp
    time_us_ = meas_package.timestamp_;
    
    //-----------------
    // Prediction step
    //-----------------
    Prediction(dt);
    
    //-----------------
    // Update step
    //-----------------
    UpdateHandler(meas_package);
    
    // Print the output
    if (VERBOSE) {
      cout << "x_ = " << x_ << endl;
      cout << "P_ = " << P_ << endl;
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
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Defines the sensor that wil be used for the update step
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateHandler(MeasurementPackage meas_package){
  // Update the filter with the appropriate sensor equations
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR &&
      use_radar_) {
    // Update using radar
    UpdateRadar(meas_package);
    
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER &&
             use_laser_) {
    // Update using lidar
    UpdateLidar(meas_package);
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
