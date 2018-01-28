#include "ukf.h"
#include "tools.h"
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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // augmented sigma points.
  sigma_points_ = MatrixXd(7, 15);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5, 5);

  // TODO put this stuff in Q
  // Process noise standard deviation longitudinal acceleration in m/s^2
  auto std_a_ = 6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  auto std_yawdd_ = 6;

  Q_ = MatrixXd(2, 2);
  Q_ << std_a_, 0,
        0, std_yawdd_;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  auto std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  auto std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  auto std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  auto std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  auto std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_, 0, 0,
              0, std_radphi_, 0,
              0, 0, std_radrd_;
}

UKF::~UKF() {}

VectorXd polar_to_cartesian(VectorXd polar)
{
  double rho = polar[0], phi = polar[1], rho_dot = polar[2];
  auto px = sqrt((rho*rho) / dbz_guard(1 - tan(phi))); // divide by zero
  auto py = sqrt((rho*rho) - (px * px));
  auto cartesian = VectorXd(4);
  cartesian << px, py, 0.0, 0.0;
  return cartesian;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage m) {
  if (!is_initialized_)
  {
    if (m.sensor_type_ == MeasurementPackage::RADAR) {
      x_ = polar_to_cartesian(m.raw_measurements_);
    }
    else if (m.sensor_type_ == MeasurementPackage::LASER) {
      x_ << m.raw_measurements_(0), m.raw_measurements_(1), 0, 0;
    }

    previous_timestamp_ = m.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }


}

void UKF::generate_sigma_points()
{
  auto lambda = -4;
  MatrixXd P_aug = MatrixXd::Zero(7, 7);
  P_aug.block(0, 0, 5, 5) = P_;
  P_aug.block(5, 5, 2, 2) = Q_;
  P_aug = lambda * P_aug;
  MatrixXd root = P_aug.llt().matrixL();

  sigma_points_.col(0) = x_;

  int i = 1;
  for (; i < (sigma_points_.cols() - 1)/2; i++)
  {
    sigma_points_.col(i) = x_ + root.col(i);
  }
  for (; i < (sigma_points_.cols() - 1)/2; i++)
  {
    sigma_points_.col(i) = x_ - root.col(i);
  }
}

VectorXd UKF::process_model(const VectorXd &state)
{
  return VectorXd(5);
}

void UKF::predict_sigma_points()
{

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  generate_sigma_points();

  predict_sigma_points();

  // Calculate mean and covariance.
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
