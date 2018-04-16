#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "poly_utils.h"
#include <iostream>
#include <fstream>
#include "json.hpp"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 12;
double dt = 0.05;
// idx_delay is the number of acctuations that will stay the same as the initial value
// the first acctuation is the current value,
// the next 2 (0.1s delay / 0.05 dt = 2) will be fixed to be the same as the current value
size_t idx_delay = 3;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//const double ref_v = 50; // will read ref_v from file

// The vars would have the x, y, psi, v, cte, epsi for each timestamp
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;

// the current values of delta and a,
// and the values we will set for each timestamp except the last one
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N;
// only constrain the first idx_delay delta and a values
size_t delta_constrain_start = delta_start;
size_t a_constrain_start = delta_constrain_start + idx_delay;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  nlohmann::json configJson;
  FG_eval(Eigen::VectorXd coeffs) {
    this->coeffs = coeffs;
    std::ifstream configFile("../config.in");  // read all tuning parameters from file
    configFile >> configJson;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += configJson["cte_coeff"] * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += configJson["epsi_coeff"] * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += configJson["v_coeff"] *  CppAD::pow(vars[v_start + t] - configJson["ref_v"], 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += configJson["delta_coeff"] * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += configJson["a_coeff"] *  CppAD::pow(vars[a_start + t], 2);
      fg[0] += configJson["vdelta_coeff"] * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += configJson["d_delta_coeff"] * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += configJson["d_a_coeff"] * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Constrain to the initial states
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // constrain the first idx_delay delta and a values to the current state
    for (int t = 0; t < idx_delay; t++) {
      fg[1 + delta_constrain_start + t] = vars[delta_start + t];
      fg[1 + a_constrain_start + t] = vars[a_start + t];
    }

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // the actuation at index t is for time t not t + 1.
      AD<double> delta1 = vars[delta_start + t];
      AD<double> a1 = vars[a_start + t];

      // desired position and angle
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // constrain the new state to be the one calculate from previous states
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta1 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a1 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta1 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // An additional pair of delta and a values are added for the current states
  // so that the first few timestamps can be constrained to the current values
  size_t n_vars = N * 6 + N * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6 + idx_delay * 2;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // initial state
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];
  vars[delta_start] = state[6];
  vars[a_start] = state[7];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0E5;
    vars_upperbound[i] = 1.0E5;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // set the constraints for the delta and a values in the first idx_delay points
  for (int i = delta_constrain_start; i < a_constrain_start; i++) {
    constraints_lowerbound[i] = state[6];
    constraints_upperbound[i] = state[6];
  }
  for (int i = a_constrain_start; i < n_constraints; i++) {
    constraints_lowerbound[i] = state[7];
    constraints_upperbound[i] = state[7];
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          55\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // The variables can be accessed with
  // `solution.x[i]`.
  std::vector<double> solution_results(N * 2 + 5);
  // Return the predicted car trace
  for(int i = 0; i < N; ++i){
    solution_results[i] = solution.x[x_start + i];
    solution_results[i + N] = solution.x[y_start + i];
  }
  // Return the first actuator values after the delay
  solution_results[N * 2] = solution.x[delta_start + idx_delay];
  solution_results[1 + N * 2] = solution.x[a_start + idx_delay];
  solution_results[2 + N * 2] = solution.x[x_start + idx_delay - 1];
  solution_results[3 + N * 2] = solution.x[y_start + idx_delay - 1];
  solution_results[4 + N * 2] = solution.x[psi_start + idx_delay - 1];
  return solution_results;
}
