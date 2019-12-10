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
#include "Eigen-3.3.7/Eigen/QR"
#include "mpc.h"

#define POLYFIT_ORDER 2

using namespace std;
using Eigen::VectorXd;

double speed = 0;
vector <geometry_msgs::PointStamped> path_points;

void speedCallback(const std_msgs::Float64::ConstPtr& msg);
void pathCallback(const nav_msgs::Path::ConstPtr& msg);
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);
//Convert v, delta and psi to Twist for use with f1 simulator
geometry_msgs::Twist getTwist(double v, double delta, double psi);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Publisher f1sim_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher target_speed = nh.advertise<std_msgs::Float64>("target_speed", 1000);
  ros::Publisher steering_angle = nh.advertise<std_msgs::Float64>("steering_angle", 1000);
  ros::Publisher optimal_path = nh.advertise<nav_msgs::Path>("optimal_path", 1000);
  ros::Publisher polynomial_path = nh.advertise<nav_msgs::Path>("polynomial_path", 1000);
  ros::Publisher drive = nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1000);
  // order of the forces - [Ffx, Ffy, Frx, Fry, |Ff|, |Fr|]
  ros::Publisher tyre_forces = nh.advertise<std_msgs::Float64MultiArray>("tyre_forces", 1000);
  ros::Subscriber speed_sub = nh.subscribe("speed", 1000, speedCallback);
  ros::Subscriber closest_path_points = nh.subscribe("closest_path_points", 1000, pathCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;


  Params p;
  double max_steering_angle;
  double v_max, v_min;
  int loop_rate;

  pnh.param("loop_rate", loop_rate, 10);
  pnh.param("prediction_horizon", p.prediction_horizon, 10);
  pnh.param("dt", p.dt, 0.2);
  pnh.param("max_steering_angle", max_steering_angle, 0.44);
  pnh.param("w_cte", p.w_cte, 100.0);
  pnh.param("w_eps", p.w_eps, 100.0);
  pnh.param("w_delta_var", p.w_delta_var, 2000.0);
  pnh.param("w_delta", p.w_delta, 20.0);
  pnh.param("w_v", p.w_v, 100.0);
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

  while(ros::ok())
  {

    if(path_points.empty())
    {
      ros::spinOnce();
      cout << path_points.size() << endl;
      rate.sleep();
      continue;
    }
    //get current state info
    listener.lookupTransform("/skidpad", "/base_link", ros::Time(0), transform);
    vector<geometry_msgs::PointStamped> path_points_base_link;
    for(unsigned int i = 0; i < path_points.size(); ++i)
    {
      geometry_msgs::PointStamped pointS;
      listener.transformPoint("/base_link", path_points[i], pointS);
      path_points_base_link.push_back(pointS);
    }

    VectorXd x(path_points.size());
    VectorXd y(path_points.size());
    for(unsigned int i = 0; i < path_points_base_link.size(); ++i)
    {
      x(i) = path_points_base_link[i].point.x;
      y(i) = path_points_base_link[i].point.y;
    }
    VectorXd trajectory_coefficients_result;
    trajectory_coefficients_result = polyfit(x, y, POLYFIT_ORDER);
    std::vector <double> trajectory_coefficients;

    trajectory_coefficients.push_back(trajectory_coefficients_result[0]);
    trajectory_coefficients.push_back(trajectory_coefficients_result[1]);
    trajectory_coefficients.push_back(trajectory_coefficients_result[2]);

    p.trajectory_coefficients = trajectory_coefficients;

    std::vector<double> state(p.state_vars);
    state[0] = 0; //x
    state[1] = 0; //y
    tf::Quaternion base_link_rot_qaternion = transform.getRotation();
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 rotation_mat(base_link_rot_qaternion);
    rotation_mat.getRPY(roll, pitch, yaw, 1);
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

    std_msgs::Float64 target_speed_msg;
    std_msgs::Float64 steering_angle_msg;
    nav_msgs::Path optimal_path_msg;
    nav_msgs::Path polynomial_path_msg;

    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp.sec = ros::Time::now().sec;
    drive_msg.header.stamp.nsec = ros::Time::now().nsec;
    drive_msg.drive.steering_angle = controls.delta;
    drive_msg.drive.speed = controls.velocity;
    drive_msg.drive.acceleration = controls.acceleration;


    target_speed_msg.data = controls.velocity;
    steering_angle_msg.data = controls.delta;
    optimal_path_msg = controls.predicted_path;
    polynomial_path_msg = controls.polynomial_path;

    drive.publish(drive_msg);
    target_speed.publish(target_speed_msg);
    steering_angle.publish(steering_angle_msg);
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

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  path_points.clear();
  for(unsigned int i = 0; i < msg->poses.size(); ++i)
  {
    geometry_msgs::PointStamped pointS;
    pointS.point = msg->poses[i].pose.position;
    pointS.header.frame_id = "/skidpad";
    path_points.push_back(pointS);
  }
}


// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  double x_max = xvals[0];
  int valid_points = 1;
  for(int i = 1; i < xvals.size(); ++i)
  {
    if(xvals[i] > xvals[i - 1])
    {
      ++valid_points;
      x_max = xvals[i];
    }
  }


  Eigen::MatrixXd A(valid_points, order + 1);

  for (int i = 0; i < valid_points; ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < valid_points; ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  VectorXd yvals1(valid_points);
  for(int i = 0; i < valid_points; ++i)
  {
    yvals1[i] = yvals[i];
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals1);

  return result;
}
