#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "fg_eval.h"
//#include "solver.h"


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

//Mpc* mpcPtr;

void speedCallback(const std_msgs::Float32::ConstPtr& msg);
bool updated;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    ros::Publisher target_speed = nh.advertise<std_msgs::Float64>("target_speed", 1000);
    ros::Publisher steering_angle = nh.advertise<std_msgs::Float64>("steering_angle", 1000);
    ros::Publisher optimal_path = nh.advertise<nav_msgs::Path>("optimal_path", 1000);
    ros::Subscriber speed = nh.subscribe("speed", 1000, speedCallback);


    Params p;

    nh.param("~prediction_horizon", p.prediction_horizon, 10);
    nh.param("~delta_time", p.delta_time, 0.05);
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



    //Mpc mpc(p);
    //mpcPtr = &mpc;
    updated = 0;
    while(ros::ok())
    {
      if(updated)
      {
        //publish
        //
      }
    }
    return 0;
}


void speedCallback(const std_msgs::Float32::ConstPtr& msg)
{
    double speed = msg->data;
    updated = 1;
    //mpcPtr->updateSpeed(speed);
}
