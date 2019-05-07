#include "solver.h"

std::vector<double> Solver::Solve(const VectorXd &state, const VectorXd &pathCoeffs)
{
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  /*
    Initiate variables describing a state of the car
  */
  //Position of the car on the map
  double x = state[0];
  double y = state[1];
  //Angle with respect to the x(?) axis
  double psi = state[2];
  //Velocity with respect to the map
  double v = state[3];
  //Distance from the desired path
  double cte = state[4];
  //Heading offset from the desired path
  double epsi = state[5];

  /*
   * Initiate indexing variables //TODO ogarnac zeby to bylo dzielone z fg_eval
   */
  /*size_t x_start = 0;
  size_t y_start = x_start + N;
  size_t psi_start = y_start + N;
  size_t v_start = psi_start + N;
  size_t cte_start = v_start + N;
  size_t epsi_start = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start = delta_start + N - 1;*/

  //Set the number of model variables
  size_t n_vars = STATE_VARS * N + ACTUATORS_VARS * (N - 1);
  //Set the number of constraints
  size_t n_constraints = STATE_VARS * N;

  //Initial value of the independent variables
  //SHOULD BE 0 besides initial state
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i)
  {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  /*
   * Set lower and upper limits for state variables
   */
  for (size_t i = 0; i < delta_start; ++i)
  {
    vars_lowerbound[i] = 1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  /*
   * Set initial state variables
   */
  vars_lowerbound[x_start] = x;
  vars_upperbound[x_start] = x;
  vars_lowerbound[y_start] = y;
  vars_upperbound[y_start] = y;
  vars_lowerbound[psi_start] = psi;
  vars_upperbound[psi_start] = psi;
  vars_lowerbound[v_start] = v;
  vars_upperbound[v_start] = v;
  vars_lowerbound[cte_start] = cte;
  vars_upperbound[cte_start] = cte;
  vars_lowerbound[epsi_start] = epsi;
  vars_upperbound[epsi_start] = epsi;

  // The upper and lower limits of delta are set to -25 and 25
  // degrees in radians
  for (size_t i = delta_start; i < a_start; ++i)
  {
    vars_lowerbound[i] = -1*MAX_MOD_DELTA;
    vars_upperbound[i] = MAX_MOD_DELTA;
  }

  //Acceleration/decceleration upper and lower limits
  for (size_t i = a_start; i < n_vars; ++i)
  {
    vars_lowerbound[i] = MAX_DECCELERATION;
    vars_upperbound[i] = MAX_DECCELERATION;
  }

  // Lower and upper limits for the constraints.
  // The constraints are defined as difference between a state variable provided by
  // the ipopt optimizer and a state variable resulting from the model
  // equations, so they should be always equal to 0, because we only want to do
  // calculations for states that can actually happen considering the initial
  // state and bounds on actuators
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Object that computes the cost function f and the constraints g_i
  FG_eval fg_eval(pathCoeffs);

  //options for IPOPT optimizer
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // Object that is used to store the solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // Optimize the cost function
  CppAD::ipopt::solve<Dvector, FG_eval>(
    options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);

  // Display the cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return only the first actuator values
  std::vector<double> ret_val;
  ret_val.push_back(solution.x[delta_start]);
  ret_val.push_back(solution.x[a_start]);

  // Also return the optimal positions to display predicted path in rviz
  for (size_t i = 1; i < N; ++i)
  {
    ret_val.push_back(solution.x[x_start + i]);
    ret_val.push_back(solution.x[y_start + i]);
  }

  return ret_val;
}
