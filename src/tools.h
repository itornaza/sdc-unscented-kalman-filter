#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor
  */
  Tools();

  /**
  * Destructor
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                         const vector<VectorXd> &ground_truth);

  /**
   * A helper method normalize the angle into the range -π to π
   */
  void normalizeAngle(double &);
  
  /**
   * A helper method to calculate the Normalized Innovation Squared (NIS)
   * for all sensors
   */
  float CalculateNIS(const VectorXd &z_diff, const MatrixXd &S);
  
  
  /**
   * A helper method to report the NIS values
   */
  void ReportNIS(int timesteps, int NIS_lidar_over, int NIS_lidar_cntr,
                 int NIS_radar_over, int NIS_radar_cntr);
  
};

#endif /* TOOLS_H_ */
