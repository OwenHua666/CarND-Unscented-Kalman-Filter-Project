#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;
  // VectorXd hx_;
  ///* state covariance matrix
  MatrixXd P_;

  ///* process covariance matrix
  MatrixXd Q_;

  ///* state transition matrix
  MatrixXd F_;

  ///* sigma point matrix
  MatrixXd Xsig_;

  ///* augmented sigma point matrix
  MatrixXd Xsig_aug_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* measurement data
  VectorXd z_;

  ///* predicted measurement data
  VectorXd z_pred_;

  ///* the measurement corresponding to sigma points
  MatrixXd Zsig_;

  ///* predicted measurement covariance
  MatrixXd S_;

  ///* Radar measurement noise
  MatrixXd R_radar_;

  ///* Lidar measurement noise
  MatrixXd R_lidar_;

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

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  /// Number of sigma points
  int n_sig_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* The NIS
  double NIS;

  ///* previous timestamp
  long previous_timestamp_;

  ///# Step count
  int step_;

  ///* small value limit
//  double p_x_min_;
//  double p_y_min_;
  double p_x_min_;
  double p_y_min_;

  Eigen::MatrixXd RMSE_NIS_Collect;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

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
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

   /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void AugmentSigmaPoints();

  /**
  * Generate the augmented sigma points
  */
  void PredictSigmaPoints(double delta_t);

  /**
  * predict the sigma points in the next time step
  * @param delta_t The time step
  */
  void PredictMeanAndCovariance();

  /**
  * predict the mean and covariance
  */
  void PredictRadarMeasurement();

  /**
  * measurement function for Radar
  */
  void PredictLidarMeasurement();

  /**
  * measurement function for Lidar
  */

  void UpdateState(int n_z, bool is_radar);

  /**
  * UKF Update
  */

  void UpdateMeasurement(MeasurementPackage meas_package);
};
#endif /* UKF_H */
