#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#define MIN_SPEED 0.5

// set ref_v
void cmd_speedCallback(const std_msgs::Float64::ConstPtr& msg);
// set model_v
void speedCallback(const std_msgs::Float64::ConstPtr& msg);
// returns torque value
double pidController();

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

int main(int argc, char** argv){

  ros::init(argc, argv, "model_cmd_converter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber model_v_sub = nh.subscribe<std_msgs::Float64>("speed", 1000, speedCallback);
  ros::Subscriber ref_v_sub = nh.subscribe<std_msgs::Float64>("cmd_speed", 1000, cmd_speedCallback);
  ros::Publisher model_control_pub = nh.advertise<std_msgs::Float64MultiArray>("model_control", 1000);
  pnh.param("KP", KP, 1.0);
  pnh.param("KI", KI, 0.0);
  pnh.param("KD", KD, 0.0);

  std_msgs::Float64MultiArray model_control;
  model_control.data.resize(2);
  ros::Rate rate(100);

  while(ros::ok()){
    ros::spinOnce();
    model_control.data[0] = 0; // delta
    model_control.data[1] = pidController(); // torque
    ROS_INFO("torque: %lf, speed: %lf\n", model_control.data[1], ref_v);
    model_control_pub.publish(model_control);
    rate.sleep();
  }

  return 0;
}


void cmd_speedCallback(const std_msgs::Float64::ConstPtr& msg){

  ref_v = msg->data;
  if(ref_v < MIN_SPEED) ref_v = MIN_SPEED;
  //ROS_INFO("ref_v from mpc: %lf\n", ref_v);
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
  dt = 0.01;
  error = ref_v - model_v;
  integral = integral + error * dt;
  derivative = (error - previous_error) / dt;
  acceleration = KP * error + KI * integral + KD * derivative;
  torque = acceleration * 1300 * 0.3;
  previous_error = error;
  return torque;
}
