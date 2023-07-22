#include "Sensor/sensor.hpp"
#include "StateEstimator/state_estimator.hpp"
#include "TrajectoryGenerator/constants.hpp"
#include "TrajectoryGenerator/trajectory_gen.hpp"
#include "constants.hpp"
#include "control_system.hpp"
#include "plant.hpp"

#include "matplot/matplot.h"

namespace plt = matplot;

int main() {

  // mass spring dampener system
  int n = 2;
  double c_damp = 0.2;
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
  // sensors.push_back(new WhiteSensor(0, 0.1));
  // sensors.push_back(new WhiteSensor(0, 0.2));
  sensors.push_back(new PinkSensor(10, 0.1, 0.1));
  sensors.push_back(new PinkSensor(10, 0.1, 0.1));

  // init estimator
  NULLEstimator no_estimator;

  // KF Matrices
  Eigen::MatrixXd F, G, Q, H, R;

  F.setIdentity(n, n);
  F = A * DT + F;

  G = (Eigen::Vector2d() << 1, 0).finished();

  Q.resize(n, n);
  Q(0, 0) = 0.0001;
  Q(1, 1) = 0.0001;

  H.setIdentity(n, n);

  R.resize(n, n);
  R(0, 0) = 0.01;
  R(1, 1) = 0.04;

  KalmanFilter kf(F, G, Q, H, R);

  // create reference trajectory
  lander_data lander_params;
  trajectory_constraint traj_cons{
      .pos_0 = (Eigen::Vector3d() << 2400, 450, -330).finished(),
      // .vel_0 = (Eigen::Vector3d() << 0, 0, 0).finished(),
      .vel_0 = (Eigen::Vector3d() << -10, -40, -10).finished(),
      .gamma = M_PI / 6};
  env_data world{.PLANET_W = EARTH_W, .PLANET_G = EARTH_G};
  SplineTrajectory traj_gen(lander_params, traj_cons, world);

  // form Control System
  Plant *plant_ptr = &msd_plant;
  StateEstimator *est_ptr = &kf;
  TrajectoryGenerator *traj_ptr = &traj_gen;
  ControlSystem msd(plant_ptr, sensors, est_ptr, traj_ptr);
  Eigen::VectorXd state = (Eigen::Vector2d() << 2.0, 3.0).finished();

  // simulate
  double tF = 100.0;
  msd.simulate(tF, state, DT, false, true);
  msd.plot_data();
  plt::save("MSD.png");

  return 0;
}
