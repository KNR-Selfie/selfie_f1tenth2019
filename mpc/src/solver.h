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

using CppAD::AD;
using Eigen::VectorXd;

class Solver{

  //prediction horizon steps
  size_t N;

public:
  std::vector<double> Solve(const VectorXd &state, const VectorXd &coeffs);

};


#endif //SOLVER_H
