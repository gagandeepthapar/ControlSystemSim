#include "plant.hpp"
#include "ODESolver/ode_solver.hpp"
#include "sim_constants.hpp"

/*
 * Set ODE Solver
 */

void Plant::init_ode_system(ODE_TYPE ode_type, int num_states,
                            double dt = SIM_CONST::DT) {
  t_step = dt;
  switch (ode_type) {
  case RKF45:
    m_ode = new ODESOLVER::RKF45(
        std::bind(&Plant::apply_transition, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3),
        dt, 1e-8, num_states);
    break;

  case FE:
    m_ode = new ODESOLVER::ForwardEuler(
        std::bind(&Plant::apply_transition, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3),
        dt, 1e-8, num_states);
    break;
  }
  return;
}

/*
 * Compose Class Methods
 */

LinearSystem::LinearSystem(Eigen::MatrixXd &state_A, Eigen::MatrixXd &inp_B,
                           Eigen::MatrixXd &state_C, Eigen::MatrixXd &inp_D,
                           int num_states, int num_inp,
                           ODE_TYPE ode_type = RKF45)
    : A(state_A), B(inp_B), C(state_C), D(inp_D) {
  init_ode_system(ode_type, num_states);
  this->num_states = num_states;
  this->num_inputs = num_inputs;
}

Eigen::VectorXd LinearSystem::apply_transition(double time,
                                               Eigen::VectorXd state,
                                               Eigen::VectorXd input) {
  // returns x_dot
  // Linear Time INVARIANT system (A, B, C, D not functions of t)
  return A * state + B * input;
}

Eigen::VectorXd LinearSystem::update(Eigen::VectorXd state,
                                     Eigen::VectorXd input, double time) {
  // integrate xdot
  return m_ode->solve(state, {0, t_step}, input);
}
