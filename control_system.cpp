#include "control_system.hpp"
#include "constants.hpp"
#include "matplot/matplot.h"
#include <__chrono/duration.h>
#include <chrono>

namespace plt = matplot;

ControlSystem::ControlSystem(Plant *system_plant,
                             std::vector<Sensor *> sensor_suite,
                             StateEstimator *state_estimator)
    : plant(system_plant), sensor_set(sensor_suite),
      estimator(state_estimator) {
  this->init_system();
}

void ControlSystem::init_system() {
  this->num_states = this->plant->num_states;
  this->num_inputs = this->plant->num_inputs;
}

void ControlSystem::init_sim(double time, double t_step) {
  // calc num steps
  this->plant->t_step = t_step;
  this->num_steps = (int)time / t_step + 2;
  // allocate space for signal data
  this->time_bus.resize(this->num_steps);
  this->truth_bus.resize(this->num_states, this->num_steps);
  this->measurement_bus.resize(this->num_states, this->num_steps);
  this->estimation_bus.resize(this->num_states, this->num_steps);
  this->controller_bus.resize(this->num_states, this->num_steps);
  this->acutator_bus.resize(this->num_states, this->num_steps);
}

void ControlSystem::print_log(double time) {
  // print useful info
  std::cout << "Number of States: " << this->num_states << std::endl;
  std::cout << "Number of Inputs: " << this->num_inputs << std::endl;
  std::cout << "Number of Steps: " << this->num_steps << std::endl;
  std::cout << "Solution Time [us]: " << this->sol_time << std::endl;
  std::cout << "FTRT Ratio: " << time / ((double)this->sol_time * 1e-6)
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
  (void)this->init_sim(time, t_step);

  // create intermediary state vectors
  // Eigen::VectorXd truth_state(this->num_states);
  Eigen::VectorXd meas_state(this->num_states);
  Eigen::VectorXd est_state(this->num_states);
  Eigen::VectorXd prev_state(this->num_states);
  Eigen::VectorXd actuation(1);
  actuation.setZero();

  // if initial state is known then push initial_state else push zeros
  meas_state =
      known_state ? true_state : Eigen::VectorXd::Zero(true_state.size());

  est_state =
      known_state ? true_state : Eigen::VectorXd::Zero(true_state.size());

  // push initial states
  this->time_bus[0] = 0;
  this->truth_bus.col(0) = true_state;
  this->measurement_bus.col(0) = meas_state;
  this->estimation_bus.col(0) = est_state;

  // iterate through system dynamics
  double t = 0;
  int col_idx = 1;
  while ((t <= time) && (t_step > SMALL)) {
    // apply control

    // update state (use true state for propagation)
    true_state = this->plant->update(truth_bus.col(col_idx - 1), actuation, t);
    truth_bus.col(col_idx) = true_state;

    // measure state vector
    for (Sensor *sensor : this->sensor_set) {
      int state_id = (int)sensor->sensor_id / 1000 - 1;
      meas_state[state_id] = sensor->sample(true_state[state_id]);
    }
    measurement_bus.col(col_idx) = meas_state;

    // estimate state
    est_state = this->estimator->estimate(meas_state, actuation);
    estimation_bus.col(col_idx) = est_state;

    // update time
    this->time_bus[col_idx] = t;

    // check time bounds
    t_step = (t + t_step > time) ? (time - t) : (t_step);
    t += t_step;
    col_idx += 1;
  }
  auto stop = std::chrono::high_resolution_clock::now();
  this->sol_time =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start)
          .count();

  // print log if verbose mode
  if (verbose) {
    this->print_log(time);
  }

  return;
}

void ControlSystem::plot_data() {
  std::vector<int> all_state(this->num_states);
  std::iota(all_state.begin(), all_state.end(), 0);
  return this->plot_data(all_state, false);
}

void ControlSystem::plot_data(bool error) {
  std::vector<int> all_state(this->num_states);
  std::iota(all_state.begin(), all_state.end(), 0);
  return this->plot_data(all_state, error);
}

void ControlSystem::plot_data(std::vector<int> state_num) {
  return this->plot_data(state_num, false);
}

void ControlSystem::plot_data(std::vector<int> state_num, bool error) {

  // plot
  plt::tiledlayout(state_num.size(), (int)error + 1);
  for (int idx : state_num) {
    // time hist
    auto next = plt::nexttile();
    plt::hold(true);
    plt::plot(this->time_bus, (Eigen::VectorXd)this->truth_bus.row(idx));
    plt::plot(this->time_bus, (Eigen::VectorXd)this->measurement_bus.row(idx),
              "r--");
    plt::plot(this->time_bus, (Eigen::VectorXd)this->estimation_bus.row(idx),
              "g");
    plt::hold(false);
    plt::legend({"True", "Measured", "Estimated"});
    plt::title("State " + std::to_string(idx));

    if (error) {
      // error
      next = plt::nexttile();
      plt::hold(true);
      plt::plot(this->time_bus,
                (Eigen::VectorXd)(this->truth_bus.row(idx) -
                                  this->measurement_bus.row(idx)),
                "r--");
      plt::plot(this->time_bus,
                (Eigen::VectorXd)(this->truth_bus.row(idx) -
                                  this->estimation_bus.row(idx)),
                "g");
      plt::hold(false);
      plt::legend({"Measured", "Estimated"});
      plt::title("State " + std::to_string(idx) + " Error");
    }
  }
  plt::show();
  return;
}
