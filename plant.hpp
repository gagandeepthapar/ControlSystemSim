#ifndef PLANT_HPP
#define PLANT_HPP

#include "ODESolver/ode_solver.hpp"
#include "eigen3/Eigen/Core"
#include <iostream>
#include <string>
// enum to specify ode type
enum ODE_TYPE { FE, RKF45 };

class Plant {

public:
  // returns x_dot
  virtual Eigen::VectorXd update(Eigen::VectorXd state, Eigen::VectorXd input,
                                 double time) = 0;

protected:
  void init_ode_system(ODE_TYPE ode_type, int num_states, double dt);

  // applies xdot = f(x, u, t)
  virtual Eigen::VectorXd apply_transition(double time, Eigen::VectorXd state,
                                           Eigen::VectorXd input) = 0;

protected:
  ODESOLVER::ODESolver *m_ode;

public:
  int num_states;
  int num_inputs;
  double t_step;
};

class LinearSystem : public Plant {
protected:
  Eigen::VectorXd apply_transition(double time, Eigen::VectorXd state,
                                   Eigen::VectorXd input) override;

public:
  LinearSystem(Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::MatrixXd &C,
               Eigen::MatrixXd &D, int num_states, int num_inp,
               ODE_TYPE ode_type);

  Eigen::VectorXd update(Eigen::VectorXd state, Eigen::VectorXd input,
                         double time) override;

public:
  // Eigen::VectorXd x, x_dot, y;
  Eigen::MatrixXd A, B, C, D;
};
//
// class NonLinearSystem : public Plant {
// private:
//   Eigen::VectorXd apply_transition(Eigen::VectorXd state, Eigen::VectorXd
//   input,
//                                    double time);
//
// public:
//   Eigen::VectorXd update(Eigen::VectorXd state, Eigen::VectorXd input,
//                          double time) override;
//
// private:
//   // xdot = f(x, u, t);
//   Eigen::VectorXd (*m_f_xut)(Eigen::VectorXd, Eigen::VectorXd, double);
//
//   // y = g(x, u, t);
//   Eigen::VectorXd (*m_g_xut)(Eigen::VectorXd, Eigen::VectorXd, double);
// };
//
#endif // !PLANT_HPP
