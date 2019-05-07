#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "mpc.h"

#define POLYFIT_ORDER 2

using namespace std;
using Eigen::VectorXd;

double speed;
vector <geometry_msgs::PointStamped> path_points;

void speedCallback(const std_msgs::Float32::ConstPtr& msg);
void pathCallback(const nav_msgs::Path::ConstPtr& msg);
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;
  ros::Publisher target_speed = nh.advertise<std_msgs::Float64>("target_speed", 1000);
  ros::Publisher steering_angle = nh.advertise<std_msgs::Float64>("steering_angle", 1000);
  ros::Publisher optimal_path = nh.advertise<nav_msgs::Path>("optimal_path", 1000);
  ros::Subscriber speed = nh.subscribe("speed", 1000, speedCallback);
  ros::Subscriber closest_path_points = nh.subscribe("closest_path_points", 1000, pathCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;


  Params p;
  int loop_rate;

  nh.param("~prediction_horizon", p.prediction_horizon, 10);
  nh.param("~delta_time", p.delta_time, 0.05);
  nh.param("~loop_rate", loop_rate, 10);
  nh.param("~max_mod_delta", p.max_mod_delta, 0.44);
  nh.param("~max_acceleration", p.max_acceleration, 1.0);
  nh.param("~max_decceleration", p.max_decceleration, -1.0);
  nh.param("~cte_weight", p.cte_weight, 100);
  nh.param("~epsi_weight", p.epsi_weight, 100);
  nh.param("~v_weight", p.v_weight, 15);
  nh.param("~delta_weight", p.delta_weight, 2000);
  nh.param("~a_weight", p.a_weight, 100);
  nh.param("~diff_delta_weight", p.diff_delta_weight, 100);
  nh.param("~diff_a_weight", p.diff_a_weight, 10);
  nh.param("~ref_v", p.ref_v, 2);


  Mpc mpc(p);
  ros::Rate rate(loop_rate);
  double x, y, orientation;
  while(ros::ok())
  {
    if(path_points.empty())
    {
      continue;
    }
    //get current state info
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    vector<geometry_msgs::PointStamped> path_points_base_link;
    for(unsigned int i = 0; i < path_points.size())
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

    VectorXd state(6);
    //dalej uzupelnienie state, getControls i publish

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
    pointS.header.frameID = "/map";
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
