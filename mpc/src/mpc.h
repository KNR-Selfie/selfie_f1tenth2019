#ifndef MPC_H
#define MPC_H
#include "fg_eval.h"
#include "solver.h"
#include <vector>

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
};

//Struct that is returned by MPC
struct Controls
{
  double acceleration;
  double delta;
}

class MPC{

  // Number of steps in the prediction horizon
  size_t N;
  // Distance from the rear axis to the center of mass
  size_t Lf;
  // Duration of one timestep
  double dt
  // Indexing variables that are used by fg_eval and solver
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

public:
  // Constructor
  MPC(Params params);
  // Main method of the MPC
  Controls getControls(Eigen::VectorXd pathCoeffs, const VectorXd &state);

}


#endif //MPC_H
