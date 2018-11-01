#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
private:
  /**
   *   Generate sigma points:
   *  @param x : State vector.
   *  @param P : Covariance matrix.
   *  @param sig_spreading: Sigma points spreading parameter.
   *  @param n_sig: Sigma points dimension.
   */
  MatrixXd GenerateSigmaPoints(VectorXd x, MatrixXd P, double sig_spreading, int n_sig);

  /**
   * Predits sigma points.
   * @param Xsig : Sigma points to predict.
   * @param delta_t : Time between k and k+1 in s
   * @param n_x : State dimension.
   * @param n_sig : Sigma points dimension.
   * @param nu_am : Process noise standard deviation longitudinal acceleration in m/s^2
   * @param nu_yawdd : Process noise standard deviation yaw acceleration in rad/s^2
   */
  MatrixXd PredictSigmaPoints(MatrixXd Xsig, double delta_t, int n_x, int n_sig, double nu_am, double nu_yawdd);

  /**
   *  Normalized the component `index` of the vector `vector` to be inside [-M_PI, M_PI] interval.
   */
  void NormalizeAngleOnComponent(VectorXd vector, int index);
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

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

    /*Process noise standard deviation*/
  double pn_std_long_acce;// longitudinal acceleration in m/s^2
  double pn_std_yaw_acce;//  yaw acceleration in rad/s^2

  /*Laser measurement noise standard deviation */
  double lm_std_px; // position1 in m
  double lm_std_py;// position2 in m

  /*Radar measurement noise standard deviation */
  double rm_std_radius; // radius in m
  double rm_std_angle;// angle in rad
  double rm_std_radius_change ;  // radius change in m/s

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int state_dimention;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double sig_spreading;

  ///* Sigma points dimension
  int n_sig_;

  ///* Radar measurement noise covariance matrix
  MatrixXd R_radar_;

  ///* Lidar measurement noise covariance matrix
  MatrixXd R_lidar_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  UKF();

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
};

#endif /* UKF_H */