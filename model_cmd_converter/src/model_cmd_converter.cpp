#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "selfie_msgs/MPCControl.h"
#include "selfie_msgs/ModelControl.h"
#include <cmath>
#define MIN_SPEED 0.5

// set target control and compute torque with PID
void target_controlCallback(const selfie_msgs::MPCControl::ConstPtr& msg);
// set model_v
void speedCallback(const std_msgs::Float64::ConstPtr& msg);

// returns torque value
double pidController();

// 
double steering_angle = 0;
// speed received from mpc
double ref_v = MIN_SPEED;
// speed of the model
double model_v = 0;
// previous error for pid
double previous_error = 0;
// integral for pid
double integral = 0;
//time of last pid call
double prev_t;
// pid params
double KP, KI, KD;

ros::Publisher model_control_pub;

int main(int argc, char** argv){

  ros::init(argc, argv, "model_cmd_converter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber model_v_sub = nh.subscribe<std_msgs::Float64>("model_speed", 1000, speedCallback);
  ros::Subscriber target_control_sub = nh.subscribe<selfie_msgs::MPCControl>("target_control", 1000, target_controlCallback);
  model_control_pub = nh.advertise<selfie_msgs::ModelControl>("model_control", 1000);
  pnh.param("KP", KP, 1.0);
  pnh.param("KI", KI, 0.0);
  pnh.param("KD", KD, 0.0);

  prev_t = ros::Time::now().toSec();
  ros::spin();

  return 0;
}


void target_controlCallback(const selfie_msgs::MPCControl::ConstPtr& msg){

  selfie_msgs::ModelControl model_control;
  model_control.steering_angle = msg->steering_angle; // delta
  ref_v = msg->speed;
  if(ref_v < MIN_SPEED) ref_v = MIN_SPEED; 
  model_control.torque = pidController(); // torque
  //ROS_INFO("torque: %lf, speed: %lf\n", model_control.data[1], ref_v);
  //ROS_INFO("ref_v from mpc: %lf\n", ref_v);
  model_control_pub.publish(model_control);

  return;
}

void speedCallback(const std_msgs::Float64::ConstPtr& msg){

  model_v = msg->data;
  if(isnan(model_v)) exit(1);
  //ROS_INFO("ref_v from mpc: %lf\n", ref_v);
  return;
}

double pidController(){

  double dt, derivative, error, acceleration, torque;
  dt = ros::Time::now().toSec() - prev_t;
  prev_t = ros::Time::now().toSec();
  //ROS_INFO("dt = %f", dt);
  error = ref_v - model_v;
  integral = integral + error * dt;
  derivative = (error - previous_error) / dt;
  acceleration = KP * error + KI * integral + KD * derivative;
  torque = acceleration * 1300 * 0.3;
  previous_error = error;
  return torque;
}
