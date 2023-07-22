#include "Sensor/sensor.hpp"
#include "StateEstimator/state_estimator.hpp"
#include "TrajectoryGenerator/gfold.hpp"
#include "TrajectoryGenerator/traj_constants.hpp"
#include "TrajectoryGenerator/trajectory_gen.hpp"
#include "control_system.hpp"
#include "plant.hpp"

#include "matplot/matplot.h"

namespace plt = matplot;

int main() {

  // mars landing system
  const int n = 6;

  // set S_Omega matrix (Eqn. 2)
  Eigen::MatrixXd S_Omega = (Eigen::Matrix3d() << 0, -MARS_W[2], MARS_W[1],
                             MARS_W[2], 0, -MARS_W[0], -MARS_W[1], MARS_W[0], 0)
                                .finished();

  // set A matrix (Eqn. 2)
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
  A.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
  A.bottomLeftCorner<3, 3>() = -(S_Omega * S_Omega);
  A.bottomRightCorner<3, 3>() = -2 * S_Omega;

  // set B matrix (Eqn. 2)
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
  B.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(n, n);

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n, 1);

  // init plant
  LinearSystem lander(A, B, C, D, n, 1, FE);

  // setup sensors
  std::vector<Sensor *> sensors;
  for (int i = 0; i < n; i++) {
    sensors.push_back(new PinkSensor(10, 1, 1));
  }

  // init estimator
  NULLEstimator no_estimator;

  // KF Matrices
  Eigen::MatrixXd F, G, Q, H, R;

  F.setIdentity(n, n);
  F = A * DT + F;

  G = (Eigen::Vector<double, n>{} << 1, 1, 1, 1, 1, 1).finished();

  Q.resize(n, n);
  Q(0, 0) = 0.0001;
  Q(1, 1) = 0.0001;

  H.setIdentity(n, n);

  R.resize(n, n);
  R(0, 0) = 0.01;
  R(1, 1) = 0.04;

  KalmanFilter kf(F, G, Q, H, R);
  NULLEstimator no_est;

  // create reference trajectory
  lander_data lander_params;
  trajectory_constraint traj_cons{
      .pos_0 = (Eigen::Vector3d() << 2400, 450, -330).finished(),
      // .vel_0 = (Eigen::Vector3d() << 0, 0, 0).finished(),
      .vel_0 = (Eigen::Vector3d() << -40, -10, 10).finished(),
      .gamma = M_PI / 6};
  env_data world{.PLANET_W = MARS_W, .PLANET_G = MARS_G};
  SplineTrajectory traj_gen(lander_params, traj_cons, world);

  // form Control System
  Plant *plant_ptr = &lander;
  StateEstimator *est_ptr = &no_est;
  TrajectoryGenerator *traj_ptr = &traj_gen;

  Eigen::VectorXd state =
      (Eigen::Vector<double, 6>{} << traj_cons.pos_0, traj_cons.vel_0)
          .finished();

  std::cout << "INIT PROPER" << std::endl;
  std::cout << state << std::endl;
  ControlSystem lander_sys(plant_ptr, sensors, est_ptr, traj_ptr);

  // simulate
  double tF = 50.0;
  lander_sys.simulate(tF, state, DT, false, true);

  Eigen::VectorXd &time = lander_sys.time_bus;
  Eigen::MatrixXd &truth = lander_sys.estimation_bus;
  Eigen::MatrixXd &cmd = lander_sys.reference_bus;
  const Eigen::MatrixXd &ref = traj_gen.state();

  Eigen::VectorXd x = (Eigen::VectorXd)truth.row(0);
  Eigen::VectorXd y = (Eigen::VectorXd)truth.row(1);
  Eigen::VectorXd z = (Eigen::VectorXd)truth.row(2);

  Eigen::VectorXd cmd_x = (Eigen::VectorXd)cmd.row(0);
  Eigen::VectorXd cmd_y = (Eigen::VectorXd)cmd.row(1);
  Eigen::VectorXd cmd_z = (Eigen::VectorXd)cmd.row(2);

  Eigen::VectorXd ref_x = (Eigen::VectorXd)ref.row(0);
  Eigen::VectorXd ref_y = (Eigen::VectorXd)ref.row(1);
  Eigen::VectorXd ref_z = (Eigen::VectorXd)ref.row(2);

  plt::hold(true);
  plt::plot3(ref_z, ref_y, ref_x, "r--")->line_width(2);
  plt::plot3(cmd_z, cmd_y, cmd_x, "g--")->line_width(2);
  plt::plot3(z, y, x)->line_width(2);
  plt::hold(false);
  plt::legend({"True Reference", "Commanded Reference", "Experimental"});
  plt::title(traj_gen.name);
  plt::show();

  plt::tiledlayout(3, 1);

  for (int i = 0; i < 3; i++) {
    plt::nexttile();
    plt::hold(true);
    plt::plot(time, (Eigen::VectorXd)ref.row(i));
    plt::plot(time, (Eigen::VectorXd)cmd.row(i));
    plt::plot(time, (Eigen::VectorXd)truth.row(i));
    plt::hold(false);
    plt::legend({"True Reference", "Commanded Reference", "Experimental"});
  }

  // lander_sys.plot_data();
  // plt::save("MSD.png");
  plt::show();

  return 0;
}
