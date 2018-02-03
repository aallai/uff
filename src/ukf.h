#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <climits>

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

  ///* state covariance matrix
  MatrixXd P_;

  ///* Process Noise
  MatrixXd Q_;

  ///* Lidar measurement noise.
  MatrixXd R_laser_;

  ///* Radar measurement noise.
  MatrixXd R_radar_;

  ///* predicted sigma points matrix
  MatrixXd sigma_points_;

  std::vector<double> weights_;

  ///* time when the state is true, in us
  long long previous_timestamp_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;


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

private:

  VectorXd weighted_mean(const MatrixXd &m);
  MatrixXd weighted_covariance(const MatrixXd &m, const VectorXd &u, int normalize_index = INT_MAX);

  MatrixXd state_to_lidar();
  MatrixXd state_to_radar();
  void generate_weights();
  void generate_sigma_points();
  void predict_sigma_points(double t);
  void update(const MatrixXd &measurement_prediction,
              const VectorXd &measurement_mean,
              const MatrixXd &measurement_covariance,
              const VectorXd &innovation);

  // Takes state+noise vector in R^7 and returns predicted state vector in R^5.
  VectorXd process_model(VectorXd state, double t);
};

#endif /* UKF_H */
