#ifndef UKF_H
#define UKF_H
#include "Eigen/Dense"
#include "measurement_package.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class UKF {
public:
  bool is_initialized;
  bool use_lidar;
  bool use_radar;

  long last_timestamp;

  double sd_a;
  double sd_yaw_dd;
  double sd_las_px;
  double sd_las_py;
  double sd_rad_r;
  double sd_rad_phi;
  double sd_rad_r_d;

  double dt_threshold;

  int n_x;
  int n_aug;
  int n_z_radar;
  int n_z_lidar;
  double lambda;
  double scale;

  double dt;
  double dt2;
  
  VectorXd x;
  MatrixXd P;

  MeasurementPackage meas_package;

  VectorXd weights;

  VectorXd Xaug;
  MatrixXd Paug;

  MatrixXd Xsig_aug;
  MatrixXd Xsig_pred;

  MatrixXd Zsig_radar;
  VectorXd Zpred_radar;
  MatrixXd S_radar;
  MatrixXd R_radar;
  VectorXd Zdiff_radar;
  MatrixXd Tc_radar;

  MatrixXd Zsig_lidar;
  VectorXd Zpred_lidar;
  MatrixXd S_lidar;
  MatrixXd R_lidar;
  VectorXd Zdiff_lidar;
  MatrixXd Tc_lidar;

  double NIS;

  UKF();
  virtual ~UKF();

  void ProcessMeasurement(MeasurementPackage meas_package_in);
  void InitializeMeasurement();
  void InitializeWeights();
  void UpdateTime();
  void Predict();
  void UpdateLidar();
  void UpdateRadar();

  void AugmentedSigmaPoints();
  void SigmaPointPrediction();
  double ConstrainAngle(double angle);
  void PredictMeanAndCovariance();
  void PredictRadarMeasurement();  
  void UpdateRadarState();
  void PredictLidarMeasurement();
  void UpdateLidarState();
  void CalculateRadarNIS();
  void CalculateLidarNIS();

  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, 
                         const vector<VectorXd> &ground_truth);

}; 

#endif /* UKF_H */
