#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define AUG_STATE_DIM 7
#define STATE_DIM 5
#define LAMBDA 3

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
  x_ = VectorXd(AUG_STATE_DIM);

  // augmented sigma points.
  sigma_points_ = MatrixXd(x_.size(), 1 + 2 * x_.size());

  // initial covariance matrix
  P_ = MatrixXd::Identity(STATE_DIM, STATE_DIM);

  generate_weights();

  // TODO put this stuff in Q
  // Process noise standard deviation longitudinal acceleration in m/s^2
  auto std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  auto std_yawdd_ = 0.2;

  Q_ = MatrixXd(2, 2);
  Q_ << std_a_*std_a_, 0,
        0, std_yawdd_*std_yawdd_;

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

void UKF::generate_weights()
{
  weights_.push_back(LAMBDA/double(LAMBDA + AUG_STATE_DIM));

  for (int i = 1; i < sigma_points_.cols(); i++)
  {
    weights_.push_back(1/(2*double(LAMBDA+AUG_STATE_DIM)));
  }
}

VectorXd polar_to_cartesian(VectorXd polar)
{
  double rho = polar[0], phi = polar[1], rho_dot = polar[2];
  auto px = sqrt((rho*rho) / dbz_guard(1 - tan(phi))); // divide by zero
  auto py = sqrt((rho*rho) - (px * px));
  VectorXd cartesian(AUG_STATE_DIM);
  cartesian << px, py, 0, 0, 0, 0, 0;
  return cartesian;
}

/**
 * @param {MeasurementPackage} m The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage m) {

  if (!is_initialized_)
  {
    if (m.sensor_type_ == MeasurementPackage::RADAR) {
      x_ = polar_to_cartesian(m.raw_measurements_);
    }
    else if (m.sensor_type_ == MeasurementPackage::LASER) {
      x_ << m.raw_measurements_(0), m.raw_measurements_(1), 0, 0, 0, 0, 0;
    }

    previous_timestamp_ = m.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  Prediction((m.timestamp_ - previous_timestamp_) / 1000000.0);

  cout << "After predict" << endl;
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl << endl;

  if (m.sensor_type_ == MeasurementPackage::RADAR)
  {
    cout << "Radar" << endl;
    UpdateRadar(m);
  }
  else
  {
    cout << "Lidar" << endl;
    UpdateLidar(m);
  }

  cout << "After update" << endl;
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl << endl;

  previous_timestamp_ = m.timestamp_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  generate_sigma_points();

  predict_sigma_points(delta_t);

  x_ = weighted_mean(sigma_points_);
  P_ = weighted_covariance(sigma_points_.block(0, 0, STATE_DIM, sigma_points_.cols()), x_.head(STATE_DIM), 3);
}

void UKF::generate_sigma_points()
{
  auto lambda = LAMBDA - AUG_STATE_DIM;
  MatrixXd P_aug = MatrixXd::Zero(x_.size(), x_.size());
  P_aug.block(0, 0, STATE_DIM, STATE_DIM) = P_;
  P_aug.block(STATE_DIM, STATE_DIM, Q_.rows(), Q_.cols()) = Q_;

  P_aug = lambda * P_aug;
  MatrixXd root = P_aug.llt().matrixL();

  sigma_points_.col(0) = x_;

  int i = 1;
  for (; i <= 7; i++)
  {
    sigma_points_.col(i) = x_ + root.col(i - 1);
  }
  for (; i < sigma_points_.cols(); i++)
  {
    sigma_points_.col(i) = x_ - root.col(i - x_.size() - 1);
  }
}

VectorXd UKF::process_model(VectorXd state, double t)
{
  //cout << "State: " << state << endl;

  double px = state(0);
  double py = state(1);
  double v = state(2);
  double yaw = state(3);
  double yaw_rate = state(4);
  double accel = state(5);
  double yaw_accel = state(6);

  // Zero out noise from state.
  state(5) = 0.0;
  state(6) = 0.0;

  VectorXd step(7);
  step.setZero();
  VectorXd noise(7);
  noise.setZero();

  if (fabs(yaw_rate) > 0.001)
  {
    //cout << "yaw rate nonzero" << endl;
    step(0) = (v/yaw_rate) * (sin(yaw + yaw_rate*t) - sin(yaw));
    step(1) = (v/yaw_rate) * (cos(yaw) - cos(yaw + yaw_rate*t));
  }
  else
  {
    //cout << "yaw rate zero" << endl;
    //cout << "t: " << t << endl;
    //cout << "v: " << v << endl;
    //cout << "yaw: " << yaw << endl;
    step(0) = v*t*cos(yaw);
    step(1) = v*t*sin(yaw);
  }

  //cout << "Step: " << step << endl;

  step(3) = yaw_rate*t;

  noise << 0.5*(t*t)*cos(yaw)*accel,
           0.5*(t*t)*sin(yaw)*accel,
           t*accel,
           0.5*(t*t)*yaw_accel,
           t*yaw_accel,
           0,
           0;

  //cout << "Noise: " << noise << endl;

  VectorXd ret = state + step + noise;
  ret(3) = normalize(ret(3));
  return ret;
}

void UKF::predict_sigma_points(double t)
{
  for (int i = 0; i < sigma_points_.cols(); i++)
  {
    sigma_points_.col(i) = process_model(sigma_points_.col(i), t);
  }
}

VectorXd UKF::weighted_mean(const MatrixXd &m)
{
  VectorXd mean(m.rows());
  mean.setZero();

  for (int i = 0; i < m.cols(); i++)
  {
    mean += weights_[i] * m.col(i);
  }

  return mean;
}

MatrixXd UKF::weighted_covariance(const MatrixXd &m, const VectorXd &u, int normalize_index)
{
  MatrixXd covariance(u.size(), u.size());
  covariance.setZero();

  for (int i = 0; i < m.cols(); i++)
  {
    VectorXd diff = m.col(i) - u;

    if (normalize_index != INT_MAX)
      diff(normalize_index) = normalize(diff(normalize_index));

    covariance += weights_[i] * diff * diff.transpose();
  }

  return covariance;
}

MatrixXd UKF::state_to_lidar()
{
  MatrixXd T(2, x_.size());

  T << 1, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0;

  return T * sigma_points_;
}

MatrixXd UKF::state_to_radar()
{
  MatrixXd radar(3, sigma_points_.cols());

  for (int i = 0; i < sigma_points_.cols(); i++)
  {
    VectorXd point = sigma_points_.col(i);
    double px = point(0), py = point(1), v = point(2), phi = point(3);

    radar(0, i) = sqrt(px*px + py*py);
    radar(1, i) = atan2(py, px);
    radar(2, i) = (px*cos(phi)*v + py*sin(phi)*v) / dbz_guard(sqrt(px*px + py*py));
  }

  return radar;
}

void UKF::update(const MatrixXd &measurement_prediction,
                 const VectorXd &measurement_mean,
                 const MatrixXd &measurement_covariance,
                 const VectorXd &innovation)
{
  // Calculate cross correlation.
  MatrixXd corr(x_.size(), innovation.size());
  corr.setZero();

  for (int i = 0; i < sigma_points_.cols(); i++)
  {
    corr += weights_[i]*(sigma_points_.col(i) - x_)*(measurement_prediction.col(i) - measurement_mean).transpose();
  }

  MatrixXd K = corr * measurement_covariance.inverse();

  x_ += K * innovation;
  P_ -= (K * measurement_covariance * K.transpose()).block(0, 0, STATE_DIM, STATE_DIM);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} m
 */
void UKF::UpdateLidar(MeasurementPackage m) {

  // Transform sigma points into measurement space.
  MatrixXd measurement_prediction = state_to_lidar();

  VectorXd measurement_mean = weighted_mean(measurement_prediction);
  MatrixXd measurement_covariance = weighted_covariance(measurement_prediction, measurement_mean) + R_laser_;

  update(measurement_prediction,
         measurement_mean,
         measurement_covariance,
         m.raw_measurements_ - measurement_mean);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} m
 */
void UKF::UpdateRadar(MeasurementPackage m) {
  MatrixXd measurement_prediction = state_to_radar();

  VectorXd measurement_mean = weighted_mean(measurement_prediction);
  MatrixXd measurement_covariance = weighted_covariance(measurement_prediction, measurement_mean, 2) + R_radar_;
  VectorXd innovation = m.raw_measurements_ - measurement_mean;

  // Normalize angles.
  innovation(1) = atan2(sin(innovation(1)), cos(innovation(1)));

  update(measurement_prediction,
         measurement_mean,
         measurement_covariance,
         innovation);
}
