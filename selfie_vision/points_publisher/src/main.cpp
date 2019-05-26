#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <selfie_msgs/Points.h>
using namespace rs2;
using namespace std;
bool DEBUG_MODE = false;
class PointsPublisher
{
public:
  PointsPublisher()
  {
    sub = n.subscribe("/camera/depth/color/points", 1, &PointsPublisher::cloud_callback, this);
    pub = n.advertise<selfie_msgs::Points>("obstacle_points", 1);
    int z;
    if (n.getParam("debug_points_publisher", z) == 1)
      DEBUG_MODE = true;
    else
      DEBUG_MODE = false;
  }
  geometry_msgs::Point *pixelTo3DPoint(const sensor_msgs::PointCloud2 cloud_msg, int *u, int *v, int a, int b)
  {
    double sumx = 0;
    double sumy = 0;
    double sumz = 0;
    int width = cloud_msg.width;
    int height = cloud_msg.height;
    geometry_msgs::Point *p = new geometry_msgs::Point[a * b];
    selfie_msgs::Points points;
    bool error = false;
    for (int i = 0; i < a; i++)
    {
      for (int j = 0; j < b; j++)
      {
        // get width and height of 2D point cloud data
        // Convert from u (column / width), v (row/height) to position in array
        // where X,Y,Z data starts
        int arrayPosition = v[i] * cloud_msg.row_step + u[j] * cloud_msg.point_step;
        if (DEBUG_MODE)
        {
          cout << "arrayPosition: " << arrayPosition << endl;
        }
        // compute position in array where x,y,z data start
        int arrayPosX = arrayPosition + cloud_msg.fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + cloud_msg.fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + cloud_msg.fields[2].offset; // Z has an offset of 8
        if (DEBUG_MODE)
        {
          cout << "arrayPosX: " << arrayPosX << endl;
          cout << "arrayPosY: " << arrayPosY << endl;
          cout << "arrayPosZ: " << arrayPosZ << endl;
        }
        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;

        memcpy(&X, &cloud_msg.data[arrayPosX], sizeof(float));
        memcpy(&Y, &cloud_msg.data[arrayPosY], sizeof(float));
        memcpy(&Z, &cloud_msg.data[arrayPosZ], sizeof(float));
        // put data into the point p
        if (DEBUG_MODE)
        {
          sumx += X;
          sumy += Y;
          sumz += Z;
        }
        p[i].x = X;
        p[i].y = Y;
        p[i].z = Z;
        if (X == 0 && Y == 0 && Z == 0)
        {
          error = true;
          break;
        }
        points.points.push_back(p[i]);
      }
    }
    if (DEBUG_MODE)
    {
      cout << "X " << sumx / (a * b) << " Y " << sumy / (a * b) << " z " << sumz / (a * b) << endl;
    }
    points.size = a * b;
    if (!error)
      pub.publish(points);
    return p;
  }
  void cloud_callback(const sensor_msgs::PointCloud2 &cloud_msg)
  {
    int *x = new int[2];
    int *y = new int[2];
    x[0] = cloud_msg.width / 2;
    y[0] = cloud_msg.height / 2;
    x[1] = cloud_msg.width / 2;
    y[1] = cloud_msg.height / 2;
    pixelTo3DPoint(cloud_msg, x, y, 2, 2);
    if (DEBUG_MODE)
    {
      cout << "cloud_msg.width : " << cloud_msg.width << endl;
      cout << "cloud_msg.height: " << cloud_msg.height << endl;
      cout << "cloud_msg.row_step  : " << cloud_msg.row_step << endl;
      cout << "cloud_msg.point_step: " << cloud_msg.point_step << endl;
      cout << pixelTo3DPoint(cloud_msg, x, y, 2, 2)[0] << endl;
    }
  }

  sensor_msgs::PointCloud2 pointcloud2;
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_publisher");
  PointsPublisher pointsPublisher;
  ros::Rate rate(16);
  while (ros::ok)
  {
    rate.sleep();
    ros::spinOnce();
  }
}

