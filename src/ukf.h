#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class UKF {
public:

  //--------------
  // Scalars
  //--------------
  
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Number of sigma points
  int n_sig_;
  
  ///* Sigma point spreading parameter
  double lambda_;

  ///* Radar measurement space dimension
  int n_z_radar_;
  
  ///* Lidar measurement space dimension
  int n_z_lidar_;
  
  //-----------
  // Matrices
  //-----------
  
  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;
  
  ///* state covariance matrix
  MatrixXd P_;
  
  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;
  
  ///* Weights of sigma points
  VectorXd weights_;
  
  ///* Radar measuremenr noise R
  MatrixXd R_radar_;
  
  ///* Lidar measuremenr noise R
  MatrixXd R_lidar_;
  
  // Radar Normalized Innovation Squared (NIS)
  VectorXd NIS_radar_;
  int NIS_radar_cntr_;
  int NIS_radar_over_;
  int timesteps;
  
  // Lidar Normalized Innovation Squared (NIS)
  VectorXd NIS_lidar_;
  int NIS_lidar_cntr_;
  int NIS_lidar_over_;
  
  ///* Object for using the helper functions
  Tools tools;
  
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * Initialize
   * Performs the imitialization of the filter on the first measurement that
   * is received from either the radar or the lidar
   * @param meas_package The latest measurement data of either radar or laser
   */
  void Initialize(MeasurementPackage meas_package);
  
  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);
  
  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * UpdateHandler defines the sensor that will be used for the update step
   * @param meas_package The latest measurement data of either radar or laser
   */
  void UpdateHandler(MeasurementPackage meas_package);
  
  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */
