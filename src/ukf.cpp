#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // set the initialized flag
  is_initialized_ = false;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5,5);
  // Original State Dimension for this CTRV model
  n_x_ = 5;

  // Augmented State Dimension
  n_aug_ = 7;

  // Number of augmented sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // Sigma point spreading parameter lambda
  lambda_ = 3 - n_aug_;

  // Create sigma point matrix
  MatrixXd Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1);

  // Create augmented sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, n_sig_);

  // Predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.25;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // R_radar Matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;

  // R_lidar Matrix
  R_lidar_ = MatrixXd(2,2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  // Set the weights for all sigma points
  weights_ = VectorXd::Zero(n_sig_);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<n_sig_; i++){
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  step_ = 1;

  // set small value limit
  p_x_min_ = 0.0001;
  p_y_min_ = 0.0001;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...

  The initialization is done above this comment section
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */

// Angle Normalization
static double SNormalizeAngle(double phi)
{
    while (phi > M_PI){
            phi -= 2.*M_PI;
            //cout<<"Here"<<phi<<endl;
    }
    while (phi < -M_PI){
            phi += 2.*M_PI;
            //cout<<"Here"<<phi<<endl;
    }

    return phi;
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*********************************************************************
  *  Initialization
  *********************************************************************/
  if (!is_initialized_){
    // first measurement
    // cout << "UKF: "<< endl;

    // RMSE Collect
    RMSE_NIS_Collect = MatrixXd(1,5);

    double px;
    double py;
    double vx;
    double vy;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        CONVERT RADAR FROM POLAR TO CARTESIAN CORRDINATES AND INITIALIZE STATE
        */
        cout << "init radar"<< endl;

//        double rho = meas_package.raw_measurements_[0];
//        double phi = meas_package.raw_measurements_[1];
//        double rhodot = meas_package.raw_measurements_[2];
        const double rho = meas_package.raw_measurements_[0];
        const double phi = meas_package.raw_measurements_[1];
        const double rhodot = meas_package.raw_measurements_[2];
        px = rho*cos(phi);
        py = rho*sin(phi);
        vx = rhodot*cos(phi);
        vy = rhodot*sin(phi);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        cout << "init lidar"<<endl;

        px = meas_package.raw_measurements_[0];
        py = meas_package.raw_measurements_[1];
        vx = 0;
        vy = 0;
    }

    // Deal with small px py values
    if (fabs(px) < p_x_min_){
        px = p_x_min_;
        cout << "initial px is too small"<< endl;
    }

    if (fabs(py) < p_y_min_){
        py = p_y_min_;
        cout << "initial py is too small"<< endl;
    }

    x_ << px, py, sqrt(pow(vx,2) + pow(vy, 2)), 0, 0;
    //x_ << 0.1, 0.1, 5, 0.5, 0.2;
    previous_timestamp_ = meas_package.timestamp_;
    // Initialization down
    is_initialized_ = true;
    return;
  }

  /**************************************************************************
  * Prediction
  ***************************************************************************/

  cout << "Start Prediction" << endl;
  step_ += 1;
  cout << "Step: "<< step_ << endl;
  const double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
  cout << "dt: " << dt << endl;
  previous_timestamp_ = meas_package.timestamp_;

  // Generate Augmented sigma points
  AugmentSigmaPoints();

  // Predict
  Prediction(dt);

  cout << "End Prediction" << endl;

  /***************************************************************************
  * Update
  ***************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Update using Radar measurement model
    cout << "Radar update" << endl;

    UpdateRadar(meas_package);

  }else {
      cout << "Laser update" << endl;

    UpdateLidar(meas_package);
  }
}

/**
* Generate augmented sigma points
*/
void UKF::AugmentSigmaPoints()
{
    // Augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    // Augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // Mean state
    x_aug.fill(0.0);
    x_aug.head(5) = x_;

    // State covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = pow(std_a_, 2);
    P_aug(6,6) = pow(std_yawdd_, 2);

    // The sqrt matrix
    MatrixXd L = P_aug.llt().matrixL();

    // Augmented sigma points
    Xsig_aug_ = MatrixXd::Zero(n_aug_, n_sig_);
    Xsig_aug_.col(0) = x_aug;
    for (int i=0; i < n_aug_; i++){
        Xsig_aug_.col(i+1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }
}

/**
* Predict sigma points using the process model
*/

void UKF::PredictSigmaPoints(double delta_t)
{
    for (int i=0; i<n_sig_; i++){
        // Extract value for better interpretation
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);

        // predicted state values
        double px_p, py_p;

        // avoid to be divided by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * (sin(yaw+yawd*delta_t)-sin(yaw));
            py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
        } else {
          px_p = p_x + v*delta_t*cos(yaw);
          py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd; // no accleration on turning rate

        // add the process noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t*cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t*sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //Update the predicted sigma point
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
}

void UKF::PredictMeanAndCovariance(){

  x_.fill(0.0);

  for (int i = 0; i < n_sig_; i++){
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  P_.fill(0.0);

  for (int i = 0; i < n_sig_; i++){

    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    x_diff(3) = SNormalizeAngle(x_diff(3));
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
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
  PredictSigmaPoints(delta_t);
  PredictMeanAndCovariance();
}



/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    // Use lidar data to update the belief about the object's
    // position. Modify the state vector, x_, and covariance, P_.
    //create matrix for sigma points in measurement space
    UpdateMeasurement(meas_package);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    // Use radar data to update the belief about the object's
    // position. Modify the state vector, x_, and covariance, P_.
    UpdateMeasurement(meas_package);
}

void UKF::UpdateMeasurement(MeasurementPackage meas_package) {
    VectorXd z;
    MatrixXd z_sig;
    VectorXd z_pred;
    MatrixXd R;
    int n_z;

    if (meas_package.sensor_type_ == meas_package.RADAR) {
        n_z = 3;
        R = R_radar_;

        // Read Measurements
        double rho = meas_package.raw_measurements_[0];
        double phi = meas_package.raw_measurements_[1];
        double v = meas_package.raw_measurements_[2];

        z = VectorXd(n_z);
        z << rho, phi, v;

        // predict radar measurement
        z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
        z_pred = VectorXd(n_z);

        for (int i = 0; i < 2 * n_aug_ + 1; i++) {
            double p_x = Xsig_pred_(0, i);
            double p_y = Xsig_pred_(1, i);
            double vel = Xsig_pred_(2, i);
            double yaw = Xsig_pred_(3, i);
            double v_x = cos(yaw) * vel;
            double v_y = sin(yaw) * vel;

            // rho
            z_sig(0, i) = sqrt(p_x * p_x + p_y * p_y);

            // phi
            z_sig(1, i) = atan2(p_y, p_x);

            // rho_dot
            z_sig(2, i) = (z_sig(0, i) < 0.0001 ? (p_x * v_x + p_y * v_y) / 0.0001 : (p_x * v_x + p_y * v_y) / z_sig(0, i));
        }
    } else if (meas_package.sensor_type_ == meas_package.LASER) {
        n_z = 2;
        R = R_lidar_;

        double px = meas_package.raw_measurements_[0];
        double py = meas_package.raw_measurements_[1];

        z = VectorXd(n_z);
        z << px, py;

        z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
        z_pred = VectorXd(n_z);

        for (int i = 0; i < 2 * n_aug_ + 1; i++) {
            // measurement model
            z_sig(0, i) = Xsig_pred_(0, i);
            z_sig(1, i) = Xsig_pred_(1, i);
        }
    }


    //mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * z_sig.col(i);
    }

    //measurement covariance matrix
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    // Cross correlation matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = z_sig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();

        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = SNormalizeAngle(x_diff(3)); // normalize angle
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + R;

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // normalize angle
    if (meas_package.sensor_type_ == meas_package.RADAR) {
        z_diff(1) = SNormalizeAngle(z_diff(1));
    }



    //update state mean and covariance matrix

    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // Calculating NIS
    string sensor_type = meas_package.sensor_type_ == meas_package.RADAR ? "RADAR" : "LASER";
    NIS = z_diff.transpose() * S.inverse() * z_diff;
    cout << sensor_type << " measurement : " << NIS << endl;
}
