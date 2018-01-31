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
    // If this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // If this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    //std_a_ = 30;
    std_a_ = 5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    //std_yawdd_ = 30;
    std_yawdd_ = 1;


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


    // State dimension
    n_x_ = 5;

    // State vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    x_ = VectorXd(5);

    // State covariance matrix
    P_ = MatrixXd(5, 5);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    // Measurement noise covariance matrix for radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;

    // Measurement noise covariance matrix for lidar
    R_lidar_ = MatrixXd(2, 2);
    R_lidar_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;

    // Augmented state dimension
    n_aug_ = 7;

    // Predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;

    // Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }

    // Initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // Tools object
    tools = Tools();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // Initializing the state of x_ with the first measurement.

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            // Converting radar from polar to cartesian coordinates and initializing state.
            VectorXd cartesian = tools.ConvertFromPolarToCartesian(meas_package.raw_measurements_);
            //x_ << cartesian[0], cartesian[1], cartesian[2], cartesian[3];
            x_ << cartesian[0], cartesian[1], 0, 0, 0;
            //x_ << 0, 0, 0, 0, 0;
        } else /* if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) */ {
            // Initializing state directly from the messure.
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1],
                  0, 0, 0;
        }
        //cout << "x_: " << endl << x_ << endl;
        //cout << "P_: " << endl << P_ << endl;

        // Done with initializing, no need to predict or update
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) {
        return;
    }
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) {
        return;
    }

    /*****************************************************************************
    *  Prediction
    ****************************************************************************/

    // Computing the time elapsed between the current and previous measurements
    // delta_t - expressed in seconds
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(delta_t);

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    // Using the sensor type to perform the update step.
    // Updating the state and covariance matrices.
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    } else {
        UpdateLidar(meas_package);
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
    MatrixXd Xsig_aug = AugmentedSigmaPoints();
    SigmaPointPrediction(Xsig_aug, delta_t);
    PredictMeanAndCovariance();
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
    VectorXd z(2);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
    MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
    VectorXd z_pred = VectorXd(2);
    MatrixXd S = MatrixXd(2, 2);
    PredictLidarMeasurement(Zsig, z_pred, S);
    UpdateStateLidar(z, Zsig, z_pred, S);
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
    VectorXd z(3);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
    MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);
    VectorXd z_pred = VectorXd(3);
    MatrixXd S = MatrixXd(3, 3);
    PredictRadarMeasurement(Zsig, z_pred, S);
    UpdateStateRadar(z, Zsig, z_pred, S);
}


MatrixXd UKF::AugmentedSigmaPoints() {

    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    //create augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;
    x_aug(n_x_ + 1) = 0;

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

    return Xsig_aug;
}


void UKF::SigmaPointPrediction(const MatrixXd& Xsig_aug, const double& delta_t) {

    //predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
        }
        else {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
}


void UKF::PredictMeanAndCovariance() {

    //predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        tools.NormalizeAngle(x_diff(3));

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}


void UKF::PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // extract values for better readibility
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double v_x = cos(yaw) * v;
        double v_y = sin(yaw) * v;

        // measurement model
        double p1 = sqrt(p_x * p_x + p_y * p_y);
        Zsig(0, i) = p1;                            // rho
        Zsig(1, i) = atan2(p_y, p_x);               // phi
        Zsig(2, i) = (p_x * v_x + p_y * v_y ) / p1; // rho_dot
    }

    //mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        tools.NormalizeAngle(z_diff(1));

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + R_radar_;
}

void UKF::PredictLidarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // extract values for better readibility
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);

        // measurement model
        double p1 = sqrt(p_x * p_x + p_y * p_y);
        Zsig(0, i) = p_x; // x
        Zsig(1, i) = p_y; // y
    }

    //mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + R_lidar_;
}

void UKF::UpdateStateRadar(
    const VectorXd& z, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S) {

    // Creating matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, 3);

    // Calculating cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // Angle normalization of residual (phi)
        tools.NormalizeAngle(z_diff(1));

        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization of state (yaw)
        tools.NormalizeAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Residual
    VectorXd z_diff = z - z_pred;
    // Angle normalization of residual (phi)
    tools.NormalizeAngle(z_diff(1));

    UpdateStatestateMeanAndCovarianceMatrix(Tc, S, z_diff);
}

void UKF::UpdateStateLidar(
    const VectorXd& z, const MatrixXd& Zsig, const VectorXd& z_pred, const MatrixXd& S) {

    // Creating matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, 2);

    // Calculating cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Residual
    VectorXd z_diff = z - z_pred;

    UpdateStatestateMeanAndCovarianceMatrix(Tc, S, z_diff);
}

void UKF::UpdateStatestateMeanAndCovarianceMatrix(
    const MatrixXd& Tc, const MatrixXd& S, const VectorXd& z_diff) {

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // Updating state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}
