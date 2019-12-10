#define HAVE_STDDEF_H
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#undef HAVE_STDDEF_H
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3.7/Eigen/Core"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "mpc.h"

using CppAD::AD;
using Eigen::VectorXd;


class FG_eval {

	Params p;

public:
	typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;

	FG_eval(Params p){
		this->p = p;
	}

	void operator()(ADvector& fg, const ADvector& xi){

		size_t steering_start = p.state_vars * (p.prediction_horizon + 1);

		for(int t = 0; t < p.prediction_horizon; ++t){

			// index of steering variables
			size_t iu0 = steering_start + p.steering_vars * t;
			size_t iu1 = steering_start + p.steering_vars * (t + 1);
			// index of state variables
			size_t is0 = p.state_vars * t;
			size_t is1 = p.state_vars * (t + 1);


			AD<double> x0 = xi[p.x + is0];
			AD<double> y0 = xi[p.y + is0];
			AD<double> psi0 = xi[p.psi + is0];
			AD<double> v0 = xi[p.v + is0];
			AD<double> a0 = xi[p.a + iu0];
			AD<double> delta0 = xi[p.delta + iu0];

			AD<double> x1 = xi[p.x + is1];
			AD<double> y1 = xi[p.y + is1];
			AD<double> psi1 = xi[p.psi + is1];
			AD<double> v1 = xi[p.v + is1];
			AD<double> a1 = xi[p.a + iu1];
			AD<double> delta1 = xi[p.delta + iu1];

			AD<double> v_avg = v0 + 0.5*a0*p.dt;

			AD<double> beta0 = CppAD::atan(p.lr/(p.lf + p.lr) * CppAD::tan(delta0));
			AD<double> beta1 = CppAD::atan(p.lr/(p.lf + p.lr) * CppAD::tan(delta1));
			AD<double> at = a1;
			AD<double> an = v1*v1*CppAD::sin(beta1)/p.lr;
			AD<double> a_max = p.a_max;
			// course trajectory error
			AD<double> y_trajectory = p.trajectory_coefficients[0]
															+ x1*p.trajectory_coefficients[1]
															+ x1*x1*p.trajectory_coefficients[2];
			AD<double> psi_trajectory = p.trajectory_coefficients[1]
																+ 2.0*x1*p.trajectory_coefficients[2];

			// acceleration barrier function
			fg[0] += p.w_a * CppAD::exp(p.sigmoid_k*(CppAD::pow(at, 2) + CppAD::pow(an, 2) - CppAD::pow(a_max, 2)));
			// course trajectory error
			fg[0] += p.w_cte * CppAD::pow(y1 - y_trajectory, 2);
			// course heading error
			fg[0] += p.w_eps * CppAD::pow(psi1 - psi_trajectory, 2);
			// velocity error
			fg[0] += p.w_v * CppAD::pow(v_avg - p.v_ref, 2);
			// penalize bigger steering angles
			fg[0] += p.w_delta * CppAD::pow(delta0, 2);
			// sequential actuations
			fg[0] += p.w_delta_var * CppAD::pow(delta1 - delta0, 2);
			fg[0] += p.w_a_var * CppAD::pow(a1 - a0, 2);

			fg[1 + p.constraint_functions * t] = x1 - (x0 + v_avg * p.dt * CppAD::cos(psi0 + beta0));
			fg[2 + p.constraint_functions * t] = y1 - (y0 + v_avg * p.dt * CppAD::sin(psi0 + beta0));
			fg[3 + p.constraint_functions * t] = psi1 - (psi0 + v_avg * p.dt * CppAD::sin(beta0)/p.lr);
			fg[4 + p.constraint_functions * t] = v1 - (v0 + a0 * p.dt);

		}

		return;
	}
};


Controls MPC::mpc_solve(std::vector<double> state0, std::vector<double> state_lower,
				 std::vector<double> state_upper, std::vector<double> steering_lower,
				 std::vector<double> steering_upper, Params p){
	typedef CPPAD_TESTVECTOR( double ) Dvector;

	// number of independent state and steering variables (domain dimension for f and g)
	size_t number_of_variables = p.state_vars * (p.prediction_horizon + 1)
	                           + p.steering_vars * (p.prediction_horizon + 1);
	// number of constraints (range dimension for g)
	size_t number_of_constraints = p.constraint_functions * p.prediction_horizon;
  size_t steering_start = p.state_vars*(p.prediction_horizon + 1);
	// initial value of the independent variables
	Dvector xi(number_of_variables); for(int i = 0; i < xi.size(); ++i) xi[i] = 0;

	// lower and upper limits for independent variables
	Dvector xi_lower(number_of_variables), xi_upper(number_of_variables);

	// lower and upper limits for state variables
  for(int i = 0; i < p.state_vars; ++i){
    xi_lower[i] = state0[i]; xi_upper[i] = state0[i];
  }

	for(int i = 0; i < p.state_vars; ++i){
		for(int j = i + p.state_vars; j < steering_start; j += p.state_vars){
			xi_lower[j] = state_lower[i]; xi_upper[j] = state_upper[i];
		}
	}


	// lower and upper limits for steering variables
	for(int i = 0; i < p.steering_vars; ++i){
		for(int j = i + steering_start; j < xi.size(); j += p.steering_vars){
			xi_lower[j] = steering_lower[i]; xi_upper[j] = steering_upper[i];
		}
	}

	// lower and upper limits for g
	Dvector g_lower(number_of_constraints), g_upper(number_of_constraints);
	for(int i = 0; i < number_of_constraints; ++i){
		g_lower[i] = 0; g_upper[i] = 0;
	}

	// object that computes objective and constraints
	FG_eval fg_eval(p);

	// options
	std::string options;
	// turn off any printing
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
	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, xi, xi_lower, xi_upper, g_lower, g_upper, fg_eval, solution
	);

	// ============== DEBUG =====================
	std::cout << "cost: " << solution.obj_value << "\n";

	for(int i = 0; i < steering_start; ++i){
    if(i % p.state_vars == 0) std::cout << "\n";
		std::cout << solution.x[i] << " ";
	}
  std::cout << "\n";
  for(int i = steering_start; i < solution.x.size(); ++i){
    if(i % p.steering_vars == 0) std::cout << "\n";
		std::cout << solution.x[i] << " ";
	}
	std::cout << "\n";
	double delta1 = solution.x[steering_start + p.steering_vars];
	double beta1 = atan(p.lr/(p.lf + p.lr) * tan(delta1));
	double at = solution.x[steering_start + p.steering_vars + 1];
	double v1 = solution.x[p.state_vars + p.v];
	double an = v1*v1*sin(beta1)/p.lr;

	std::cout << "total acceleration: " << sqrt(an*an + at*at) << "\n";
	// ============== DEBUG =====================
  Controls controls;

  controls.delta = solution.x[steering_start];
  controls.velocity = solution.x[p.state_vars + p.v];
	controls.acceleration = solution.x[steering_start + p.a];

  std::vector <geometry_msgs::PoseStamped> poses(p.prediction_horizon);
  std::vector <geometry_msgs::PoseStamped> polynomial_poses(p.prediction_horizon);

  for(int i = p.state_vars; i < steering_start; i += p.state_vars){
    int j = i/p.state_vars - 1;
    poses[j].header.stamp = ros::Time::now();
    poses[j].header.frame_id = "base_link";
    poses[j].pose.position.x = solution.x[i + p.x];
    poses[j].pose.position.y = solution.x[i + p.y];
  }

  for (int i = 0; i < p.prediction_horizon; i++)
  {
    polynomial_poses[i].header.stamp = ros::Time::now();
    polynomial_poses[i].header.frame_id = "base_link";
    polynomial_poses[i].pose.position.x = i * 0.1;
    polynomial_poses[i].pose.position.y = p.trajectory_coefficients[0]
                                        + p.trajectory_coefficients[1] * (i * 0.1)
                                        + p.trajectory_coefficients[2] * (i * i * 0.01);
  }

  std::swap(controls.polynomial_path.poses, polynomial_poses);
  controls.polynomial_path.header.frame_id = "base_link";
  controls.polynomial_path.header.stamp = ros::Time::now();

  std::swap(controls.predicted_path.poses, poses);
  controls.predicted_path.header.frame_id = "base_link";
  controls.predicted_path.header.stamp = ros::Time::now();

  return controls;

}
