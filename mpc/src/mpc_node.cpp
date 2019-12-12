#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <cmath>
#include "time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <iterator>
#include <algorithm>
#include "mpc.h"

#define POLYFIT_ORDER 2

using namespace std;

double speed = 0;
bool ready = false;
Params p;

void speedCallback(const std_msgs::Float64::ConstPtr& msg);
void trajectoryCoefsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher optimal_path = nh.advertise<nav_msgs::Path>("optimal_path", 1000);
  ros::Publisher polynomial_path = nh.advertise<nav_msgs::Path>("polynomial_path", 1000);
  ros::Publisher drive = nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1000);
  // order of the forces - [Ffx, Ffy, Frx, Fry, |Ff|, |Fr|]
  ros::Subscriber speed_sub = nh.subscribe("speed", 1000, speedCallback);
  ros::Subscriber trajectory_coefs = nh.subscribe("trajectory_coefs", 1000, trajectoryCoefsCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  double max_steering_angle;
  double v_max, v_min;
  int loop_rate;

  pnh.param("loop_rate", loop_rate, 50);
  pnh.param("prediction_horizon", p.prediction_horizon, 10);
  pnh.param("dt", p.dt, 0.2);
  pnh.param("max_steering_angle", max_steering_angle, 0.44);
  pnh.param("w_cte", p.w_cte, 100.0);
  pnh.param("w_eps", p.w_eps, 100.0);
  pnh.param("w_delta_var", p.w_delta_var, 2000.0);
  pnh.param("w_delta", p.w_delta, 20.0);
  pnh.param("w_v", p.w_v, 100.0);
  pnh.param("w_a", p.w_a, 200.0);
  pnh.param("w_a_var", p.w_a_var, 100.0);
  pnh.param("v_ref", p.v_ref, 0.5);
  pnh.param("v_max", v_max, 0.5);
  pnh.param("v_min", v_min, -0.1);
  pnh.param("a_max", p.a_max, 0.8);
  pnh.param("sigmoid_k", p.sigmoid_k, 1.0);
  pnh.param("lf", p.lf, 0.25);
  pnh.param("lr", p.lr, 0.25);

  // x, y, psi, v
  p.state_vars = 4;
  // a, delta
  p.steering_vars = 2;
  p.constraint_functions = 4;

  ros::Rate rate(loop_rate);
  ros::spinOnce();

  while(ros::ok())
  {
    //get current state info

    if(!ready) {
      std::cout << "Revving up\n";
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    std::vector<double> state(p.state_vars);
    state[0] = 0; //x
    state[1] = 0; //y
    state[2] = 0; //psi
    state[3] = speed;

    MPC mpc;
    Controls controls;

    std::vector<double> state_lower{-1e9, -1e9, -1e9, v_min};

    std::vector<double> state_upper{1e9, 1e9, 1e9, v_max};

    std::vector<double> steering_lower{-max_steering_angle, -0.2};

    std::vector<double> steering_upper{max_steering_angle, 2};
    clock_t time = clock();
    controls = mpc.mpc_solve(state, state_lower, state_upper, steering_lower,
                             steering_upper, p);
    time = clock() - time;
    std::cout << "mpc_exec_time: " << (double)time/CLOCKS_PER_SEC << std::endl;

    nav_msgs::Path optimal_path_msg;
    nav_msgs::Path polynomial_path_msg;

    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp.sec = ros::Time::now().sec;
    drive_msg.header.stamp.nsec = ros::Time::now().nsec;
    drive_msg.drive.steering_angle = controls.delta;
    drive_msg.drive.speed = controls.velocity;
    drive_msg.drive.acceleration = controls.acceleration;

    optimal_path_msg = controls.predicted_path;
    polynomial_path_msg = controls.polynomial_path;

    drive.publish(drive_msg);
    optimal_path.publish(optimal_path_msg);
    polynomial_path.publish(polynomial_path_msg);

    ros::spinOnce();

    rate.sleep();
    std::cout << "speed: " << speed << std::endl;
  }
  return 0;
}


void speedCallback(const std_msgs::Float64::ConstPtr& msg)
{
  speed = msg->data;
}

void trajectoryCoefsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // totally worth it for vector length 3
  p.trajectory_coefs = msg->data;
  std::reverse(std::begin(p.trajectory_coefs), std::end(p.trajectory_coefs));
  ready = true;
}
