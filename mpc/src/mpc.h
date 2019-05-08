#ifndef MPC_H
#define MPC_H
#define HAVE_STDDEF_H
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#undef HAVE_STDDEF_H
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3.7/Eigen/Core"
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#define STATE_VARS 6
#define ACTUATORS_VARS 2
#define LF 0.2

using CppAD::AD;
using Eigen::VectorXd;

//Struct that is used to initialize the MPC
struct Params
{
  int prediction_horizon;
  int cte_weight;
  int epsi_weight;
  int v_weight;
  int delta_weight;
  int a_weight;
  int diff_delta_weight;
  int diff_a_weight;
  double delta_time;
  double max_mod_delta;
  double max_acceleration;
  double max_decceleration;
  double ref_v;
};

// Struct that is returned by MPC
struct Controls
{
  double acceleration;
  double delta;
  nav_msgs::Path predicted_path;
};

//Interface for interactions with ros
class MPC
{

public:
  // Constructor
  MPC(Params p);
  // Main method of the MPC
  Controls getControls(Eigen::VectorXd pathCoeffs, const VectorXd &state);

};

// Class that calculates the cost function f and contraints g_i
class FG_eval
{

public:
    //Coefficients of a polynomial fitted to closest path waypoints in the frame of the car
    Eigen::VectorXd pathCoeffs;
    FG_eval(Eigen::VectorXd pathCoeffs);

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars);
};

//Function that optimizes the cost function
std::vector<double> Solve(const VectorXd &state, const VectorXd &coeffs);

#endif //MPC_H
