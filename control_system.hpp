#ifndef CONTROL_SYSTEM_HPP
#define CONTROL_SYSTEM_HPP

// include custom libs
#include "ODESolver/ode_solver.hpp"
#include "Sensor/sensor.hpp"
#include "StateEstimator/state_estimator.hpp"
#include "TrajectoryGenerator/trajectory_gen.hpp"
// #include "constants.hpp"
#include "plant.hpp"

#include "eigen3/Eigen/Core"
#include <vector>

class ControlSystem {
public:
  ControlSystem(Plant *system_plant, std::vector<Sensor *> sensor_suite,
                StateEstimator *state_estimator, TrajectoryGenerator *ref_traj);
  void simulate(double time, Eigen::VectorXd state, double t_step,
                bool known_state, bool verbose);

  void plot_data();
  void plot_data(bool error);

  void plot_data(std::vector<int> state_num);
  void plot_data(std::vector<int> state_num, bool error);

private:
  void init_system();
  void init_sim(double time, double t_step);
  bool check_system();
  void print_log(double time);

public:
  // Plant (Singular)
  Plant *plant;

  // Sensor (Multiple)
  std::vector<Sensor *> sensor_set;

  // Estimator (Singular)
  StateEstimator *estimator;

  // Trajectory (Reference; Singular)
  TrajectoryGenerator *reference;

  // Signal Data
  Eigen::VectorXd time_bus;
  Eigen::MatrixXd truth_bus;
  Eigen::MatrixXd measurement_bus;
  Eigen::MatrixXd estimation_bus;
  Eigen::MatrixXd reference_bus;
  Eigen::MatrixXd controller_bus;
  Eigen::MatrixXd acutator_bus;

protected:
  // Supportive vars
  int m_num_states;
  int m_num_inputs;
  int m_num_steps;
  int m_sol_time;
};

#endif // !CONTROL_SYSTEM_HPP
