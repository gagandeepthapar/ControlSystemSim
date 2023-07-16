#include "Sensor/sensor.hpp"
#include "StateEstimator/state_estimator.hpp"
#include "constants.hpp"
#include "control_system.hpp"
#include "matplot/matplot.h"
#include "plant.hpp"
#include <cmath>

namespace plt = matplot;

int main() {

  // mass spring dampener system
  int n = 2;
  double c_damp = 0.4;
  double k_spring = 1.0;
  double mass = 1.5;

  // state dynamic matrices
  Eigen::MatrixXd A(n, n);
  A = (Eigen::Matrix2d() << 0, 1, -k_spring / mass, -c_damp / mass).finished();

  Eigen::MatrixXd B = (Eigen::Matrix<double, 2, 1>{} << 0, 1 / mass).finished();

  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(n, n);

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n, 1);

  // init plant
  LinearSystem msd_plant(A, B, C, D, n, 1, RKF45);

  // setup sensors
  std::vector<Sensor *> sensors;
  sensors.push_back(new WhiteSensor(0, 0.1));
  sensors.push_back(new WhiteSensor(0, 0.2));

  // init estimator
  NULLEstimator no_estimator;

  // KF Matrices
  Eigen::MatrixXd F, G, Q, H, R;

  F.setIdentity(n, n);
  F = A * DT + F;

  G.setZero(n, 1);

  Q.resize(n, n);
  Q(0, 0) = 0.01;
  Q(1, 1) = 0.01;

  H.setIdentity(n, n);

  R.resize(n, n);
  R(0, 0) = 0.01;
  R(1, 1) = 0.01;

  KalmanFilter kf(F, G, Q, H, R);
  std::cout << "KF INIT" << std::endl;

  // form Control System
  Plant *plant_ptr = &msd_plant;
  // StateEstimator *est_ptr = &no_estimator;
  StateEstimator *est_ptr = &kf;
  ControlSystem msd(plant_ptr, sensors, est_ptr);
  Eigen::VectorXd state = (Eigen::Vector2d() << 2.0, 3.0).finished();

  // simulate
  double tF = 100.0;
  (void)msd.simulate(tF, state, DT, true, true);
  (void)msd.plot_data();
  return 0;
}
