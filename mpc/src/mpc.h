#ifndef MPC_H
#define MPC_H

struct Params{
	double dt;
	double lr;
	double lf;
	double v_ref;
	double a_max;
	// polynomial = trajectory_coefficients[i]*x^i
	std::vector<double> trajectory_coefs;
	double w_a;
	double w_cte;
	double w_eps;
	double w_v;
	double w_delta_var;
	double w_delta;
	double w_a_var;
	double sigmoid_k;
	size_t state_vars;
	size_t steering_vars;
	int prediction_horizon;
	size_t constraint_functions;
	// order of state variables
	size_t x = 0, y = 1, psi = 2, v = 3;
	// order of steering variables
	size_t delta = 0, a = 1;
};

struct Controls{
  double delta;
  double velocity;
	double acceleration;
  nav_msgs::Path predicted_path;
  nav_msgs::Path polynomial_path;
};


//Interface for interactions with ros
class MPC{

public:

  MPC(){

  }
  // Main method of the MPC
  Controls mpc_solve(std::vector<double> state0, std::vector<double> state_lower,
  				 std::vector<double> state_upper, std::vector<double> steering_lower,
           std::vector<double> steering_upper, Params p);

};


#endif //MPC_H
