#include "control_system.hpp"
#include "TrajectoryGenerator/traj_constants.hpp"
#include "TrajectoryGenerator/trajectory_gen.hpp"
#include "matplot/matplot.h"
#include "sim_constants.hpp"
#include <__chrono/duration.h>
#include <chrono>

namespace plt = matplot;

ControlSystem::ControlSystem(Plant *system_plant,
                             std::vector<Sensor *> sensor_suite,
                             StateEstimator *state_estimator,
                             TrajectoryGenerator *ref_traj)
    : plant(system_plant), sensor_set(sensor_suite), estimator(state_estimator),
      reference(ref_traj) {
  init_system();
}

void ControlSystem::init_system() {
  m_num_states = plant->num_states;
  m_num_inputs = plant->num_inputs;
  std::cout << "PLANT STATE " << m_num_states << std::endl;
}

void ControlSystem::init_sim(double time, double t_step) {
  // calc num steps
  plant->t_step = t_step;
  m_num_steps = (int)time / t_step + 2;

  // allocate space for signal data
  time_bus.resize(m_num_steps);
  truth_bus.resize(m_num_states, m_num_steps);
  measurement_bus.resize(m_num_states, m_num_steps);
  estimation_bus.resize(m_num_states, m_num_steps);
  reference_bus.resize(m_num_states, m_num_steps);
  controller_bus.resize(m_num_states, m_num_steps);
  acutator_bus.resize(m_num_states, m_num_steps);
  std::cout << "CTRL INIT" << std::endl;
}

void ControlSystem::print_log(double time) {
  // print useful info
  std::cout << "Number of States: " << m_num_states << std::endl;
  std::cout << "Number of Inputs: " << m_num_inputs << std::endl;
  std::cout << "Number of Steps: " << m_num_steps << std::endl;
  std::cout << "Solution Time [us]: " << m_sol_time << std::endl;
  std::cout << "FTRT Ratio: " << time / ((double)m_sol_time * 1e-6)
            << std::endl;

  return;
}

void ControlSystem::simulate(double time, Eigen::VectorXd true_state,
                             double t_step = DT, bool known_state = false,
                             bool verbose = true) {

  // follow standard control loop:
  /*- pass into ODE
   *- store truth
   *- pass into sensor
   *- store measurement
   *- pass into estimator
   *- store estimate
   *- pass into controller
   *- store control signal
   *- pass into actuator
   *- store applied force
   *- pass into ODE to get next state
   *- repeat
   */

  auto start = std::chrono::high_resolution_clock::now();

  // initialize system
  (void)init_sim(time, t_step);

  // create intermediary state vectors
  // Eigen::VectorXd truth_state(m_num_states);
  Eigen::VectorXd meas_state(m_num_states);
  Eigen::VectorXd est_state(m_num_states);
  Eigen::VectorXd prev_state(m_num_states);
  Eigen::VectorXd ref_state(m_num_states);
  Eigen::VectorXd actuation(3);
  actuation.setZero();

  // if initial state is known then push initial_state else push zeros
  meas_state =
      known_state ? true_state : Eigen::VectorXd::Zero(true_state.size());

  est_state =
      known_state ? true_state : Eigen::VectorXd::Zero(true_state.size());

  // push initial states
  time_bus[0] = 0;
  truth_bus.col(0) = true_state;
  measurement_bus.col(0) = meas_state;
  estimation_bus.col(0) = est_state;

  // iterate through system dynamics
  double t = 0;
  int col_idx = 1;
  while ((t <= time) && (t_step > SIM_CONST::SMALL)) {
    // apply control
    actuation = reference->world.PLANET_G;
    // actuation = reference->input().col(col_idx);
    // std::cout << actuation << std::endl;

    // update state (use true state for propagation)
    true_state = plant->update(truth_bus.col(col_idx - 1), actuation, t);
    truth_bus.col(col_idx) = true_state;

    // measure state vector
    for (Sensor *sensor : sensor_set) {
      int state_id = (int)sensor->sensor_id / 1000 - 1;
      meas_state[state_id] = sensor->sample(true_state[state_id]);
    }
    measurement_bus.col(col_idx) = meas_state;

    // estimate state
    est_state = estimator->estimate(meas_state, actuation);
    estimation_bus.col(col_idx) = est_state;

    // get reference signal
    ref_state = reference->get_reference(est_state);
    reference_bus.col(col_idx) = ref_state;

    // update time
    time_bus[col_idx] = t;

    // check time bounds
    t_step = (t + t_step > time) ? (time - t) : (t_step);
    t += t_step;
    col_idx += 1;
  }
  auto stop = std::chrono::high_resolution_clock::now();
  m_sol_time =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start)
          .count();

  // print log if verbose mode
  if (verbose) {
    print_log(time);
  }

  return;
}

void ControlSystem::plot_data() {
  std::vector<int> all_state(m_num_states);
  std::iota(all_state.begin(), all_state.end(), 0);
  return plot_data(all_state, false);
}

void ControlSystem::plot_data(bool error) {
  std::vector<int> all_state(m_num_states);
  std::iota(all_state.begin(), all_state.end(), 0);
  return plot_data(all_state, error);
}

void ControlSystem::plot_data(std::vector<int> state_num) {
  return plot_data(state_num, false);
}

void ControlSystem::plot_data(std::vector<int> state_num, bool error) {

  // plot
  plt::tiledlayout(state_num.size(), (int)error + 1);
  for (int idx : state_num) {
    // time hist
    auto next = plt::nexttile();
    plt::hold(true);
    plt::plot(time_bus, (Eigen::VectorXd)truth_bus.row(idx));
    plt::plot(time_bus, (Eigen::VectorXd)measurement_bus.row(idx), "r--");
    plt::plot(time_bus, (Eigen::VectorXd)estimation_bus.row(idx), "g");
    plt::hold(false);
    plt::legend({"True", "Measured", "Estimated"});
    plt::title("State " + std::to_string(idx));

    if (error) {
      // error
      next = plt::nexttile();
      plt::hold(true);
      plt::plot(
          time_bus,
          (Eigen::VectorXd)(truth_bus.row(idx) - measurement_bus.row(idx)),
          "r--");
      plt::plot(time_bus,
                (Eigen::VectorXd)(truth_bus.row(idx) - estimation_bus.row(idx)),
                "g");
      plt::hold(false);
      plt::legend({"Measured", "Estimated"});
      plt::title("State " + std::to_string(idx) + " Error");
    }
  }
  plt::show();
  return;
}
