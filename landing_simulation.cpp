#include "Sensor/sensor.hpp"
#include "StateEstimator/state_estimator.hpp"
#include "TrajectoryGenerator/gfold.hpp"
#include "TrajectoryGenerator/traj_constants.hpp"
#include "TrajectoryGenerator/trajectory_gen.hpp"
#include "control_system.hpp"
#include "matplot/matplot.h"
#include "plant.hpp"
#include <algorithm>
#include <format>
#include <string>

namespace plt = matplot;
const bool ANIMATE = false;

void plot_3d(const Eigen::MatrixXd &exp, const Eigen::MatrixXd &cmd,
             const Eigen::MatrixXd &ref, const std::string name, bool animate) {

  Eigen::VectorXd x = (Eigen::VectorXd)exp.row(0);
  Eigen::VectorXd y = (Eigen::VectorXd)exp.row(1);
  Eigen::VectorXd z = (Eigen::VectorXd)exp.row(2);
  Eigen::VectorXd vx = (Eigen::VectorXd)exp.row(3);
  Eigen::VectorXd vy = (Eigen::VectorXd)exp.row(4);
  Eigen::VectorXd vz = (Eigen::VectorXd)exp.row(5);

  Eigen::VectorXd cmd_x = (Eigen::VectorXd)cmd.row(0);
  Eigen::VectorXd cmd_y = (Eigen::VectorXd)cmd.row(1);
  Eigen::VectorXd cmd_z = (Eigen::VectorXd)cmd.row(2);
  Eigen::VectorXd cmd_vx = (Eigen::VectorXd)cmd.row(3);
  Eigen::VectorXd cmd_vy = (Eigen::VectorXd)cmd.row(4);
  Eigen::VectorXd cmd_vz = (Eigen::VectorXd)cmd.row(5);

  Eigen::VectorXd ref_x = (Eigen::VectorXd)ref.row(0);
  Eigen::VectorXd ref_y = (Eigen::VectorXd)ref.row(1);
  Eigen::VectorXd ref_z = (Eigen::VectorXd)ref.row(2);
  Eigen::VectorXd ref_vx = (Eigen::VectorXd)ref.row(3);
  Eigen::VectorXd ref_vy = (Eigen::VectorXd)ref.row(4);
  Eigen::VectorXd ref_vz = (Eigen::VectorXd)ref.row(5);

  plt::axes_handle ax;
  plt::figure_handle fig = plt::figure(true);
  std::string frame_ct, old_str;
  int count_len = std::to_string(x.rows()).length() + 1;

  // store axes
  fig->tiledlayout(2, 1);
  fig->nexttile();
  auto ax1 = fig->current_axes();
  fig->nexttile();
  auto ax2 = fig->current_axes();

  // ax = fig->current_axes();
  for (int i = 0; i < z.rows(); i++) {
    // position
    ax1->clear();
    ax1->hold(true);
    ax1->plot3(z, y, x)->line_width(2);
    ax1->plot3(cmd_z, cmd_y, cmd_x)->line_width(2);
    ax1->plot3(ref_z, ref_y, ref_x)->line_width(2);
    ax1->scatter3({z(i)}, {y(i)}, {x(i)})->marker_face_color({0, 0, 0});
    ax1->hold(false);

    // apply legend, title, view angle
    auto l = plt::legend({"Experimental Path", "Commanded Reference",
                          "Ideal Trajectory", "Lander"});
    l->location(plt::legend::general_alignment::topleft);
    ax1->view(69, 13);
    ax1->title(name + ": Position");

    // velocity
    ax2->clear();
    ax2->hold(true);
    ax2->plot3(vz, vy, vx)->line_width(2);
    ax2->plot3(cmd_vz, cmd_vy, cmd_vx)->line_width(2);
    ax2->plot3(ref_vz, ref_vy, ref_vx)->line_width(2);
    ax2->scatter3({vz(i)}, {vy(i)}, {vx(i)})->marker_face_color({0, 0, 0});
    ax2->hold(false);

    // save frame
    l = plt::legend({"Experimental Path", "Commanded Reference",
                     "Ideal Trajectory", "Lander"});
    l->location(plt::legend::general_alignment::topleft);
    ax2->view(69, 13);
    ax2->title(name + ": Velocity");

    old_str = std::to_string(i);
    int min = (count_len > old_str.length()) ? old_str.length() : count_len;
    frame_ct = std::string(count_len - min, '0') + old_str;

    // save frame
    if (animate) {
      ax1->parent()->save("testgif/frame" + frame_ct + ".png");
    }
  }

  if (!animate) {
    plt::show();
  }
}

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
    sensors.push_back(new IdealSensor);
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
      .vel_0 = (Eigen::Vector3d() << 40, -10, 10).finished(),
      // .vel_0 = (Eigen::Vector3d() << -0, 1, -1).finished(),
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
  lander_sys.simulate(tF, state, DT, true, true);

  Eigen::VectorXd &time = lander_sys.time_bus;
  Eigen::MatrixXd &truth = lander_sys.estimation_bus;
  Eigen::MatrixXd &cmd = lander_sys.reference_bus;
  const Eigen::MatrixXd &ref = traj_gen.state();

  plot_3d(truth, cmd, ref, traj_gen.name, ANIMATE);

  return 0;
}
