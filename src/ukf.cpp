#include "ukf.h"
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "constants.h"
#include "tools.h"

using namespace std;
using namespace Constants;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  //--------------
  // Scalars
  //--------------
  
  // Begin with the filter in not initialized phase
  is_initialized_ = false;
  
  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

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
  n_x_ = 5;
  
  // Augmented state dimension
  n_aug_ = 7;
  
  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;
  
  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  
  // Radar measurement space
  n_z_radar_ = 3;
  
  // Lidar measurement space
  n_z_lidar_ = 2;
  
  //--------------
  // Matrices
  //--------------
  
  // Initial state vector
  x_ = VectorXd(5);
  
  // Initial covariance matrix with values optimized for minimal rmse
  P_ = MatrixXd(5, 5);
  P_ << 0.03, 0,    0,  0,    0,
        0,    0.01, 0,  0,    0,
        0,    0,    1,  0,    0,
        0,    0,    0,  0.05, 0,
        0,    0,    0,  0,    0.05;
  
  // Initializd the weights vector
  weights_ = VectorXd(n_sig_);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int ix = 1; ix < (n_sig_); ++ix) {
    weights_(ix) = 1.0 / (2.0 * (lambda_ + n_aug_));
  }
  
  // Initialize predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);
  
  // Initialize the covariance matrix for the radar
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_radar_.fill(0.0);
  R_radar_(0, 0) = std_radr_ * std_radr_;
  R_radar_(1, 1) = std_radphi_ * std_radphi_;
  R_radar_(2, 2) = std_radrd_ * std_radrd_;
  
  // Initialize the covariance matrix for the lidar
  R_lidar_ = MatrixXd(n_z_lidar_, n_z_lidar_);
  R_lidar_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  
  // Radar Normalized Innovation Squared (NIS)
  NIS_radar_ = VectorXd(1000);
  NIS_radar_cntr_ = 0;
  NIS_radar_over_ = 0;
  timesteps = 0;
  
  // Lidar Normalized Innovation Squared (NIS)
  NIS_lidar_ = VectorXd(1000);
  NIS_lidar_cntr_ = 0;
  NIS_lidar_over_ = 0;
  
  // Use the helper functions
  Tools tools;
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
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
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
    
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
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
  double dt = delta_t;
  double dt_2 = delta_t * delta_t;
  
  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  
  // Augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  
  // Sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  
  // Augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  
  // Q matrix
  MatrixXd Q(2, 2);
  Q << (std_a_ * std_a_), 0.0, 0.0, (std_yawdd_ * std_yawdd_);
  
  // Build the P_aug matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;
  
  // Create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();
  
  //----------------------------------------------
  // Step 1: Generate the Augmented sigma points
  //----------------------------------------------
  
  // First block of one col in the matrix
  Xsig_aug.col(0) = x_aug;
  
  for (int ix = 0; ix < n_aug_; ++ix) {
    // Second block of n_x_ cols in the matrix
    Xsig_aug.col(ix + 1) = x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(ix);
    
    // Third block of n_x_ cols in the matrix
    Xsig_aug.col(ix + 1 + n_aug_) =
      x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(ix);
  }
  
  //-----------------------------------------------
  // Step 2: Predict sigma points
  //-----------------------------------------------
  
  for (int ix = 0; ix < n_sig_; ix++) {
    double px = Xsig_aug(0, ix);
    double py = Xsig_aug(1, ix);
    double v = Xsig_aug(2, ix);
    double psi = Xsig_aug(3, ix);
    double psi_d = Xsig_aug(4, ix);
    double nu_a = Xsig_aug(5, ix);
    double nu_psi_dd = Xsig_aug(6, ix);
    
    // Avoid division by zero
    if (fabs(psi_d) < 0.0001) {
      px += v * cos(psi) * dt;
      py += v * sin(psi) * dt;
    } else {
      px += (v / psi_d) * (sin(psi + psi_d * dt) - sin(psi));
      py += (v / psi_d) * (-cos(psi + psi_d * dt) + cos(psi));
    }
    px += 0.5 * dt_2 * cos(psi) * nu_a;
    py += 0.5 * dt_2 * sin(psi) * nu_a;
    
    v += 0.0 + dt * nu_a;
    psi += psi_d * dt + 0.5 * dt_2 * nu_psi_dd;
    psi_d += 0.0 + dt * nu_psi_dd;
    
    Xsig_pred_(0, ix) = px;
    Xsig_pred_(1, ix) = py;
    Xsig_pred_(2, ix) = v;
    Xsig_pred_(3, ix) = psi;
    Xsig_pred_(4, ix) = psi_d;
  }
  
  //-----------------------------------------------
  // Step 3: Predicted state mean and covariance
  //-----------------------------------------------
  
  // Predicted state mean
  x_.fill(0.0);
  for (int ix = 0; ix < n_sig_; ++ix) {
    x_ += weights_(ix) * Xsig_pred_.col(ix);
  }
  
  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int ix = 0; ix < n_sig_; ++ix) {
    VectorXd x_diff = Xsig_pred_.col(ix) - x_;
    tools.normalizeAngle(x_diff(3));
    P_ += weights_(ix) * x_diff * x_diff.transpose();
  }
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
  // Matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_lidar_, n_sig_);
  
  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar_);
  
  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_lidar_, n_z_lidar_);
  
  // The current sensor measurement
  VectorXd z = meas_package.raw_measurements_;
  
  // Correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);
  
  //-----------------------------------
  // Step 1: Predict measurement
  //-----------------------------------
  
  // Transform sigma points into measurement space
  for (int ix = 0; ix < n_sig_; ++ix) {
    // Get the state vector elements
    double px = Xsig_pred_(0, ix);
    double py = Xsig_pred_(1, ix);
    
    // Assign px and py to the Zsig matrix column
    Zsig(0, ix) = px;
    Zsig(1, ix) = py;
  }
  
  // Calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int ix = 0; ix < n_sig_; ++ix) {
    z_pred += weights_(ix) * Zsig.col(ix);
  }
  
  // Calculate measurement covariance matrix S
  S.fill(0.0);
  for (int ix = 0; ix < n_sig_; ++ix) {
    VectorXd z_diff = Zsig.col(ix) - z_pred;
    S += weights_(ix) * z_diff * z_diff.transpose();
  }
  
  // Add to the predicted covariance
  S += R_lidar_;
  
  //-----------------------------------
  // Step 2: Update state
  //-----------------------------------
  
  // Cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    // Measurement difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    
    // Add to the correlation matrix
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  
  // Kalman gain K
  MatrixXd K = Tc * S.inverse();
  
  // Measurement difference
  VectorXd z_diff = z - z_pred;
  
  // Update state mean and covariance matrices
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
  
  // TODO: Calculate lidar NIS
  ++timesteps;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // Matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, n_sig_);
  
  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  
  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
  
  // The current sensor measurement
  VectorXd z = meas_package.raw_measurements_;
  
  // Correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
  
  //-----------------------------------
  // Step 1: Predict measurement
  //-----------------------------------
  
  // Transform sigma points into measurement space
  for (int ix = 0; ix < n_sig_; ++ix) {
    // Get the state vector elements
    double px = Xsig_pred_(0, ix);
    double py = Xsig_pred_(1, ix);
    double v = Xsig_pred_(2, ix);
    double psi = Xsig_pred_(3, ix);
    double psi_d = Xsig_pred_(4, ix);
    
    // Convenience variables
    double px_2 = px * px;
    double py_2 = py * py;
    
    // Divisions by zero
    if (fabs(px) < E1) {
      px = E1;
    }
    if (fabs(py) < E1) {
      py = E1;
    }
    
    // Assign the ρ, φ, ρ' to the Zsig matrix column
    Zsig(0, ix) = sqrt(px_2 + py_2);
    Zsig(1, ix) = atan2(py, px);
    Zsig(2, ix) = (px * v * cos(psi) + py * v * sin(psi)) / sqrt(px_2 + py_2);
  }
  
  // Calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int ix = 0; ix < n_sig_; ++ix) {
    z_pred += weights_(ix) * Zsig.col(ix);
  }
  
  // Calculate measurement covariance matrix S
  S.fill(0.0);
  for (int ix = 0; ix < n_sig_; ++ix) {
    VectorXd z_diff = Zsig.col(ix) - z_pred;
    tools.normalizeAngle(z_diff(1));
    S += weights_(ix) * z_diff * z_diff.transpose();
  }
  
  // Add to the predicted covariance
  S += R_radar_;
  
  //-----------------------------------
  // Step 2: Update state
  //-----------------------------------
  
  // Cross correlation matrix
  Tc.fill(0.0);
  for (int ix = 0; ix < n_sig_; ++ix) {
    VectorXd x_diff = Xsig_pred_.col(ix) - x_;
    tools.normalizeAngle(x_diff(3));
    VectorXd z_diff = Zsig.col(ix) - z_pred;
    tools.normalizeAngle(z_diff(1));
    Tc += weights_(ix) * x_diff * z_diff.transpose();
  }
  
  // Kalman gain K
  MatrixXd K = Tc * S.inverse();
  
  // Measurement difference
  VectorXd z_diff = z - z_pred;
  tools.normalizeAngle(z_diff(1));
  
  // Update state mean and covariance matrices
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
  
  // TODO: Calculate radar NIS
  NIS_radar_(NIS_radar_cntr_) = (z_diff.transpose() * S.inverse() * z_diff);
  if (NIS_radar_(NIS_radar_cntr_) > 7.8) {
    ++NIS_radar_over_;
    cout << NIS_radar_over_ << " out of " << NIS_radar_cntr_ << endl;
  }
  ++NIS_radar_cntr_;
  ++timesteps;
  
  if (timesteps >= 490) {
    cout << NIS_radar_over_ << " out of " << NIS_radar_cntr_ << endl;
  }
  
}
