#include <iostream>
#include "ukf.h"

using namespace std;

UKF::UKF() {
  // Evaluation control
  use_lidar = true;                              // Whether or not to include lidar measurements
  use_radar = true;                              // Whether or not to include radar measurements

  // Standard deviations
  sd_a = 0.6;                                    // Process noise standard deviation longitudinal acceleration in m/s^2
  sd_yaw_dd = 0.2;                               // Process noise standard deviation yaw acceleration in rad/s^2
  sd_las_px = 0.07;                              // Laser measurement noise standard deviation position1 in m
  sd_las_py = 0.07;                              // Laser measurement noise standard deviation position2 in m
  sd_rad_r = 0.1;                                // Radar measurement noise standard deviation radius in m
  sd_rad_phi = 0.001;                            // Radar measurement noise standard deviation angle in rad
  sd_rad_r_d = 0.1;                              // Radar measurement noise standard deviation radius change in m/s

  // Time related
  dt_threshold = 0.001;                          // Threshold under which time differences are set to the threshold value

  // Sizes and scaling factors
  n_x = 5;                                       // State dimension
  n_aug = 7;                                     // Augmented state dimension
  n_z_radar = 3;                                 // Number of radar measurements
  n_z_lidar = 2;                                 // Number of lidar measurements
  lambda = 3 - n_x;                              // Sigma point spreading parameter
  scale = sqrt(lambda + n_aug);                  // Scale used in sigma point spreading

  // Myriad vectors and matrices for use later
  x = VectorXd(n_x);                             // initial state vector
  P = MatrixXd(n_x, n_x);                        // initial covariance matrix

  weights = VectorXd(2*n_aug+1);                 // Weights of sigma points
  
  Xaug = VectorXd(n_aug);                        // Augmented data
  Paug = MatrixXd(n_aug, n_aug);                 // Augmented process

  Xsig_aug = MatrixXd(n_aug, 2*n_aug+1);         // Augmented sigma point matrix
  Xsig_pred = MatrixXd(n_x, 2*n_aug+1);          // Predicted sigma points (measureables only)

  Zsig_radar = MatrixXd(n_z_radar, 2*n_aug+1);   // Measurement space sigma points
  Zpred_radar = VectorXd(n_z_radar);             // Mean predicted measurements
  S_radar = MatrixXd(n_z_radar, n_z_radar);      // Measurement covariance
  R_radar = MatrixXd(n_z_radar, n_z_radar);      // Noise covariance
  Tc_radar = MatrixXd(n_x, n_z_radar);           // Correlation matrix

  Zsig_lidar = MatrixXd(n_z_lidar, 2*n_aug+1);   // Measurement space sigma points
  Zpred_lidar = VectorXd(n_z_lidar);             // Mean predicted measurements
  S_lidar = MatrixXd(n_z_lidar, n_z_lidar);      // Measurement covariance
  R_lidar = MatrixXd(n_z_lidar, n_z_lidar);      // Noise covariance
  Tc_lidar = MatrixXd(n_x, n_z_lidar);           // Correlation matrix

  // Internal control
  is_initialized = false;                        // Whether or not we are initialized
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package_in) {
  meas_package = meas_package_in;
  if (!is_initialized) {
    InitializeWeights();
    InitializeMeasurement();
  }
  if (meas_package.sensor_type == MeasurementPackage::LASER && use_lidar == true) {
    UpdateTime();
    Predict();
    UpdateLidar(); 
  }
  if (meas_package.sensor_type == MeasurementPackage::RADAR && use_radar == true) {
    UpdateTime();
    Predict();
    UpdateRadar();
  }
}

void UKF::UpdateTime() {
  dt = (meas_package.timestamp - last_timestamp) / 1000000.0;
  if (dt < dt_threshold) {
    dt = dt_threshold;
  }
  dt2 = dt * dt / 2.0;
  last_timestamp = meas_package.timestamp;
}

void UKF::InitializeWeights() {
  weights(0) = lambda / (lambda + n_aug);
  for (int i=1; i<2*n_aug+1; i++) {  
    weights(i) = 0.5 / (lambda + n_aug);
  }
}

void UKF::InitializeMeasurement() {
  last_timestamp = meas_package.timestamp;
  if (meas_package.sensor_type == MeasurementPackage::LASER && use_lidar == true) {
    x << meas_package.raw_measurements(0),
         meas_package.raw_measurements(1),
         0.0,
         0.0,
         0.0;
    P << 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0;
    R_lidar << sd_las_px*sd_las_py, 0.0, 
               0.0,                 sd_las_py*sd_las_py;
    R_radar << sd_rad_r*sd_rad_r, 0.0,                   0.0,
               0.0,               sd_rad_phi*sd_rad_phi, 0.0,
               0.0,               0.0,                   sd_rad_r_d*sd_rad_r_d;
    is_initialized = true;
  }
  if (meas_package.sensor_type == MeasurementPackage::RADAR && use_radar == true) {
    double rho = meas_package.raw_measurements(0);
    double phi = meas_package.raw_measurements(1);
    double rho_dot = meas_package.raw_measurements(2);
    x << rho * cos(phi),
         rho * sin(phi),
         rho_dot, 
         phi,
         0.0;
    P << 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0;
    R_lidar << sd_las_px*sd_las_py, 0.0, 
               0.0,                 sd_las_py*sd_las_py;
    R_radar << sd_rad_r*sd_rad_r, 0.0,                   0.0,
               0.0,               sd_rad_phi*sd_rad_phi, 0.0,
               0.0,               0.0,                   sd_rad_r_d*sd_rad_r_d;
    is_initialized = true;
  }
}

void UKF::Predict() {
  AugmentedSigmaPoints(); 
  SigmaPointPrediction();
  PredictMeanAndCovariance();
}

void UKF::UpdateLidar() {
  PredictLidarMeasurement();
  UpdateLidarState();
  CalculateLidarNIS();
}

void UKF::UpdateRadar() {
  PredictRadarMeasurement();
  UpdateRadarState();
  CalculateRadarNIS();
}

void UKF::AugmentedSigmaPoints() {
  // Generate augmented x 
  Xaug.head(5) = x;
  Xaug(5) = 0.0;
  Xaug(6) = 0.0;
  
  // Generate augmented P
  Paug.fill(0.0);
  Paug.topLeftCorner(5,5) = P;
  Paug(5,5) = sd_a * sd_a;
  Paug(6,6) = sd_yaw_dd * sd_yaw_dd; 
  
  // Square root of P
  MatrixXd Psqrt = Paug.llt().matrixL();

  // Compute sigma points and mean
  Xsig_aug.col(0) = Xaug;
  for (int i=0; i<n_aug; i++) {
    Xsig_aug.col(i+1) = Xaug + scale * Psqrt.col(i);
    Xsig_aug.col(i+1+n_aug) = Xaug - scale * Psqrt.col(i);
  }
}

void UKF::SigmaPointPrediction() {
  // Go through sigma points and predict via mapping function (with case for zero denominator)
  for (int i=0; i<2*n_aug+1; i++) {
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double psi = Xsig_aug(3,i);
    double psi_d = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_psi_dd = Xsig_aug(6,i);
    if (fabs(psi_d) > 0.001) {
      Xsig_pred(0,i) = px + v/psi_d*(sin(psi+psi_d*dt)-sin(psi)) + cos(psi)*nu_a*dt2;
      Xsig_pred(1,i) = py + v/psi_d*(cos(psi)-cos(psi+psi_d*dt)) + sin(psi)*nu_a*dt2;
      Xsig_pred(2,i) = v + nu_a*dt;
      Xsig_pred(3,i) = psi + psi_d*dt + nu_psi_dd*dt2;
      Xsig_pred(4,i) = psi_d + nu_psi_dd*dt;
    }   
    else {
      Xsig_pred(0,i) = px + v*cos(psi)*dt + cos(psi)*nu_a*dt2;
      Xsig_pred(1,i) = py + v*sin(psi)*dt + sin(psi)*nu_a*dt2;
      Xsig_pred(2,i) = v + nu_a*dt;
      Xsig_pred(3,i) = psi + psi_d*dt + nu_psi_dd*dt2;
      Xsig_pred(4,i) = psi_d + nu_psi_dd*dt;
    }
  }
}

double UKF::ConstrainAngle(double angle) {
  // Better angle constraint based on fmod
  angle = fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

void UKF::PredictMeanAndCovariance() {
  // Compute new mean x
  x.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) { 
    x += weights(i) * Xsig_pred.col(i);
  }
  
  // Compute new state covariance P
  P.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) {
    VectorXd v = Xsig_pred.col(i) - x;
    v(3) = ConstrainAngle(v(3));
    P += weights(i) * v * v.transpose();
  }
}

void UKF::PredictRadarMeasurement() {
  // Transform sigma points into measurement space
  for (int i=0; i<2*n_aug+1; i++) {
    double px = Xsig_pred(0,i);
    double py = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double psi = Xsig_pred(3,i);
    Zsig_radar(0,i) = sqrt(px*px + py*py); 
    Zsig_radar(1,i) = atan2(py,px); 
    Zsig_radar(2,i) = (px*v*cos(psi) + py*v*sin(psi)) / Zsig_radar(0,i); 
  }

  // Mean predicted measurement
  Zpred_radar.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) {
      Zpred_radar += weights(i) * Zsig_radar.col(i);
  }

  // Measurement covariance matrix S
  S_radar.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) {
    VectorXd z_diff = Zsig_radar.col(i) - Zpred_radar;
    z_diff(1) = ConstrainAngle(z_diff(1));
    S_radar += weights(i) * z_diff * z_diff.transpose();
  }
  S_radar += R_radar;
}

void UKF::UpdateRadarState() {
  // This is data coming from the measurement package 
  VectorXd z_meas = VectorXd(n_z_radar); 
  z_meas << meas_package.raw_measurements(0),
            meas_package.raw_measurements(1),
            meas_package.raw_measurements(2);

  // Cross correlation Tc
  Tc_radar.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) {
      VectorXd x_diff = Xsig_pred.col(i) - x;
      x_diff(3) = ConstrainAngle(x_diff(3));
      VectorXd z_diff = Zsig_radar.col(i) - z_meas;
      z_diff(1) = ConstrainAngle(z_diff(1));
      Tc_radar += weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc_radar * S_radar.inverse();

  // Update state mean and covariance matrix
  Zdiff_radar = z_meas - Zpred_radar;
  Zdiff_radar(1) = ConstrainAngle(Zdiff_radar(1));
  x += K * Zdiff_radar;
  P -= K * S_radar * K.transpose();
}

void UKF::PredictLidarMeasurement() {
  // Transform sigma points into measurement space
  for (int i=0; i<2*n_aug+1; i++) {
    Zsig_lidar(0,i) = Xsig_pred(0,i); 
    Zsig_lidar(1,i) = Xsig_pred(1,i); 
  }

  // Mean predicted measurement
  Zpred_lidar.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) {
      Zpred_lidar += weights(i) * Zsig_lidar.col(i);
  }

  // Measurement covariance matrix S
  S_lidar.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) {
    VectorXd z_diff = Zsig_lidar.col(i) - Zpred_lidar;
    S_lidar += weights(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  S_lidar += R_lidar;
}

void UKF::UpdateLidarState() {
  // This is data coming from the measurement package
  VectorXd z_meas = VectorXd(n_z_lidar); 
  z_meas << meas_package.raw_measurements(0), 
            meas_package.raw_measurements(1);

  // Cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z_lidar);
  Tc_lidar.fill(0.0);
  for (int i=0; i<2*n_aug+1; i++) {
    VectorXd x_diff = Xsig_pred.col(i) - x;
    x_diff(3) = ConstrainAngle(x_diff(3));
    VectorXd z_diff = Zsig_lidar.col(i) - z_meas;
    Tc_lidar += weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc_lidar * S_lidar.inverse();

  // Update state mean and covariance matrix
  Zdiff_lidar = z_meas - Zpred_lidar;
  x += K * Zdiff_lidar;
  P -= K * S_lidar * K.transpose();
}

void UKF::CalculateRadarNIS() {
  NIS = Zdiff_radar.transpose() * S_radar.inverse() * Zdiff_radar;
}

void UKF::CalculateLidarNIS() {
  NIS = Zdiff_lidar.transpose() * S_lidar.inverse() * Zdiff_lidar;
}

VectorXd UKF::CalculateRMSE(const vector<VectorXd> &estimations, 
                            const vector<VectorXd> &ground_truth) {
  VectorXd rmse = VectorXd(4);
  rmse.fill(0.0);
  for (int i=0; i<estimations.size(); i++){
    VectorXd est = estimations[i];
    double px = est(0);
    double py = est(1);
    double v = est(2);
    double psi = est(3);
    VectorXd est_cart = VectorXd(4);
    est_cart << px,
                py,
                v * cos(psi),
                v * sin(psi);
    VectorXd gt = ground_truth[i];
    VectorXd residual = est_cart - gt;
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse /= (double)estimations.size();
  return rmse.array().sqrt(); 
}
