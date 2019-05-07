#ifndef SOLVER_H
#define SOLVER_H
#define HAVE_STDDEF_H
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#undef HAVE_STDDEF_H
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3.7/Eigen/Core"
#include "fg_eval.h"
//number of variables describing a state of the car
#define STATE_VARS 6
//number of input variables for the car
#define ACTUATORS_VARS 2
//Maximum angle delta that the wheels can turn in radians
#define MAX_MOD_DELTA 0.436332
//Maximum acceleration
#define MAX_ACCELERATION 1
//Maximum decceleration
#define MAX_DECCELERATION -1
// Distance from the rear axis to the center of mass
#define Lf 0.2;
// Indexing variables that are used by fg_eval and solver

//prediction horizon steps
size_t N;
// Position on the map
size_t x_start;
size_t y_start;
// Orientation with respect to the x(?) axis
size_t psi_start;
// Velocity with respect to map
size_t v_start;
// Path position offset
size_t cte_start;
// Path heading offset
size_t epsi_start;

// Variables for actuators
// Steering angle
size_t delta_start;
// Acceleration
size_t a_start;

using CppAD::AD;
using Eigen::VectorXd;

class Solver{

public:
  std::vector<double> Solve(const VectorXd &state, const VectorXd &coeffs);

};





#endif //SOLVER_H
