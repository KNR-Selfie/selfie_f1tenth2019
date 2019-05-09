#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <cmath>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "Eigen-3.3.7/Eigen/QR"
#include "mpc.h"

#define POLYFIT_ORDER 2

using namespace std;
using Eigen::VectorXd;

double speed;
vector <geometry_msgs::PointStamped> path_points;

void speedCallback(const std_msgs::Float32::ConstPtr& msg);
void pathCallback(const nav_msgs::Path::ConstPtr& msg);
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);
//Convert v, delta and psi to Twist for use with f1 simulator
geometry_msgs::Twist getTwist(double v, double delta, double psi);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;
  ros::Publisher f1sim_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher target_speed = nh.advertise<std_msgs::Float64>("target_speed", 1000);
  ros::Publisher steering_angle = nh.advertise<std_msgs::Float64>("steering_angle", 1000);
  ros::Publisher optimal_path = nh.advertise<nav_msgs::Path>("optimal_path", 1000);
  ros::Subscriber speed_sub = nh.subscribe("speed", 1000, speedCallback);
  ros::Subscriber closest_path_points = nh.subscribe("closest_path_points", 1000, pathCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;


  Params p;
  int loop_rate;

  nh.param("prediction_horizon", p.prediction_horizon, 10);
  nh.param("delta_time", p.delta_time, 0.05);
  nh.param("loop_rate", loop_rate, 10);
  nh.param("max_mod_delta", p.max_mod_delta, 0.44);
  nh.param("max_acceleration", p.max_acceleration, 1.0);
  nh.param("max_decceleration", p.max_decceleration, -1.0);
  nh.param("cte_weight", p.cte_weight, 100);
  nh.param("epsi_weight", p.epsi_weight, 100);
  nh.param("v_weight", p.v_weight, 15);
  nh.param("delta_weight", p.delta_weight, 2000);
  nh.param("a_weight", p.a_weight, 100);
  nh.param("diff_delta_weight", p.diff_delta_weight, 100);
  nh.param("diff_a_weight", p.diff_a_weight, 10);
  nh.param("ref_v", p.ref_v, 2.0);


  MPC mpc(p);
  ros::Rate rate(loop_rate);
  double x, y, orientation;
  Controls controls;

  while(ros::ok())
  {
    if(path_points.empty())
    {
      continue;
    }
    //get current state info
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    vector<geometry_msgs::PointStamped> path_points_base_link;
    for(unsigned int i = 0; i < path_points.size(); ++i)
    {
      geometry_msgs::PointStamped pointS;
      listener.transformPoint("/base_link", path_points[i], pointS);
      path_points_base_link.push_back(pointS);
    }

    VectorXd x;
    VectorXd y;
    for(unsigned int i = 0; i < path_points_base_link.size(); ++i)
    {
      x << path_points_base_link[i].point.x;
      y << path_points_base_link[i].point.y;
    }
    VectorXd pathCoeffs;
    pathCoeffs = polyfit(x, y, POLYFIT_ORDER);

    VectorXd state(STATE_VARS);
    state<< transform.getOrigin().x(), transform.getOrigin().y();
    tf::Quaternion base_link_rot_qaternion = transform.getRotation();
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 rotation_mat(base_link_rot_qaternion);
    rotation_mat.getRPY(roll, pitch, yaw, 1);
    state<<yaw;
    state<<speed;
    state<<pathCoeffs[0];
    state<<CppAD::atan(pathCoeffs[1]);

    controls = mpc.getControls(pathCoeffs, state);

    std_msgs::Float64 target_speed_msg;
    std_msgs::Float64 steering_angle_msg;
    nav_msgs::Path optimal_path_msg;

    target_speed_msg.data = speed + controls.acceleration * p.delta_time;
    steering_angle_msg.data = controls.delta;
    optimal_path_msg = controls.predicted_path;

    geometry_msgs::Twist velocity_msg = getTwist(target_speed_msg.data, steering_angle_msg.data, yaw);

    f1sim_cmd.publish(velocity_msg);
    target_speed.publish(target_speed_msg);
    steering_angle.publish(steering_angle_msg);
    optimal_path.publish(optimal_path_msg);

    rate.sleep();
  }
  return 0;
}


void speedCallback(const std_msgs::Float32::ConstPtr& msg)
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
    pointS.header.frame_id = "/map";
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

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}

// equations from https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf
geometry_msgs::Twist getTwist(double v, double delta, double psi)
{
  double beta = atan( (LT - LF)/LT * tan(delta) );
  double ang_vel = v/(LT - LF) * sin(beta);
  double vx = v * cos(psi + beta);
  double vy = v * sin(psi + beta);

  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.linear.z = 0;

  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = ang_vel;

  return cmd_vel;
}
