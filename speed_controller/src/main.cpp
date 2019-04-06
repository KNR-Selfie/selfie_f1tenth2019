#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind/bind.hpp>
#define MSG_QUEUE_SIZE 10
using namespace std;
class a
{
  public:
  a()
  {
    pub_ = n.advertise<std_msgs::Float64>("speed_data",MSG_QUEUE_SIZE);

    lin_sub = n.subscribe("linear_offset", MSG_QUEUE_SIZE, &a::callback,this);
  }
  ros::NodeHandle n;
  ros::Publisher pub_;
  ros::Subscriber lin_sub;
  ros::Subscriber angu_sub;

  void callback1(const std_msgs::Float64& an)
  {
    this->angu.data=an.data;
  }

void callback(const std_msgs::Float64& lin)
{
  angu_sub=n.subscribe("angular_offset", MSG_QUEUE_SIZE, &a::callback1, this);
  ros::spinOnce();
  std_msgs::Float64 output;
  if(n.getParam("speed",speed)&&n.getParam("distance",dist)&&n.getParam("angle",angle)&&n.getParam("max_speed",max_speed))
  {
    output.data=speed/(dist*pow(lin.data,2)+angle*pow(angu.data,2));
    if(output.data>max_speed) output.data=max_speed;
  }
  else
  {
    output.data=1/(pow(lin.data,2)+pow(angu.data,2));
  }

  pub_.publish(output);
}
 std_msgs::Float64 angu;
  double speed=0;
  double dist=0;
  double angle=0;
  double max_speed=0;
};
int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscribe_and_publish");
  a a;
  ros::spin();
  return 0;
}
