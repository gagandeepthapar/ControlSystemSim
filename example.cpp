#include "Sensor/sensor.hpp"
#include "StateEstimator/state_estimator.hpp"
#include "constants.hpp"
#include "control_system.hpp"
#include "matplot/matplot.h"
#include "matplotlibcpp.hpp"
#include "plant.hpp"

#include <__chrono/duration.h>
#include <chrono>
#include <iostream>

namespace plt = matplot;
// namespace plt = matplotlibcpp;

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
  NULLEstimator estimator;

  // form Control System
  Plant *plant_ptr = &msd_plant;
  StateEstimator *est_ptr = &estimator;
  ControlSystem msd(plant_ptr, sensors, est_ptr);
  Eigen::VectorXd state = (Eigen::Vector2d() << 2.0, 3.0).finished();

  // simulate
  double tF = 100.0;
  auto start = std::chrono::high_resolution_clock::now();
  msd.simulate(tF, state, DT, true, true);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  std::cout << "\nTime [ms]: " << duration.count() << std::endl;
  std::cout << "FTRT Ratio: " << tF / ((double)duration.count() * 1e-6)
            << std::endl;

  plt::hold(true);
  plt::plot(msd.time_bus, (Eigen::VectorXd)msd.truth_bus.row(0));
  plt::plot(msd.time_bus, (Eigen::VectorXd)msd.measurement_bus.row(0), "r--");
  plt::plot(msd.time_bus, (Eigen::VectorXd)msd.estimation_bus.row(0), "g");
  plt::hold(false);
  plt::legend({"True", "Measured", "Estimated"});
  plt::title("MSD Position VS Time");
  plt::show();

  return 0;
}
