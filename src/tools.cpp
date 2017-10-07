#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Local variables
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  VectorXd residual;
  
  // Validate inputs
  if (estimations.size() == 0) {
    cout << "Error: The estimation vector size should not be zero" << endl;
  } else if (estimations.size() != ground_truth.size()) {
    cout << "Error: The estimation and ground truth vectors sizes are not equal"
    << endl;
  } else {
    // Accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
      residual = estimations[i] - ground_truth[i];
      residual = residual.array() * residual.array();
      rmse += residual;
    }
    
    // Calculate the mean square root
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
  }
  
  // Return the RMSE
  return rmse;
}

void Tools::normalizeAngle(double &angle) {
  while (angle > M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
}

float Tools::CalculateNIS(const VectorXd &z_diff, const MatrixXd &S) {
  return z_diff.transpose() * S.inverse() * z_diff;
}

void Tools::ReportNIS(int timesteps, int NIS_lidar_over, int NIS_lidar_cntr,
                      int NIS_radar_over, int NIS_radar_cntr) {
  if (timesteps >= 498) {
    cout << "NIS report:" << endl;
    cout  << "-> Lidar outliers: "
    << 100.0 * static_cast<float>(NIS_lidar_over) / NIS_lidar_cntr
    << "%" << endl;
    cout  << "-> Radar outliers: "
    << 100.0 * static_cast<float>(NIS_radar_over) / NIS_radar_cntr
    << "%" << endl;
  }
}
