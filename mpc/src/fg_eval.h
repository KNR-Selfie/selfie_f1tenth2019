#ifndef FG_EVAL_H
#define FG_EVAL_H


#define HAVE_STDDEF_H
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#undef HAVE_STDDEF_H
#include <iostream>
#include <vector>
#include "Eigen-3.3.7/Eigen/Core"

//Funkcja kosztu to suma kwadratow skladowych kosztow przemnozonych przez wagi:
#define CTE_WEIGHT 100
#define EPSI_WEIGHT 100
#define V_WEIGHT 15

#define DELTA_WEIGHT 2000
#define A_WEIGHT 100

//Minimize difference between sequential commands
#define DIFF_DELTA_WEIGHT 100
#define DIFF_A_WEIGHT 10


class FG_eval
{
public:

  //TODO Ta sekcja jeszcze do zgrania z MPC.h:
  //Stala promienia skretu
  //TODO dokladna wartosc dla naszego auta
  const double Lf = 2.67;
  //Prediction horizon
  size_t N;
  //Timestep length
  double dt;
  //Target velocity
  double ref_v;

  //Index of the first variable representing:
  //Position on map
  size_t x_start;
  //Position on map
  size_t y_start;
  //Orientation on map
  size_t psi_start;
  //Velocity
  size_t v_start;
  //Path position offset
  size_t cte_start;
  //Path heading offset
  size_t epsi_start;
  //Controlled variables:
  //Steering angle
  size_t delta_start;
  //Acceleration
  size_t a_start;

    //Coefficients of a polynomial fitted to closest path waypoints in the frame of the car
    Eigen::VectorXd pathCoeffs;
    FG_eval(Eigen::VectorXd pathCoeffs);

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars);
};

#endif
