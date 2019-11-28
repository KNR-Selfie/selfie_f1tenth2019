#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>

namespace gazebo
{
class ModelPush : public ModelPlugin
{
public:
    ModelPush() : ModelPlugin()
    {
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so in the gazebo_ros package)");
            return;
        }

        ROS_INFO("Hello World!");

        this->model = _parent;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ModelPush::OnUpdate, this));
        sub = nh.subscribe("drive", 1000, &ModelPush::pilotCallback, this);
        sub1 = nh.subscribe("gazebo/model_states", 1000, &ModelPush::modelStatesCallback, this);
    }
    double angle_changer(double actual, double demand)
    {
        double step = 1;
        if (actual > demand)
            return -step;
        if(demand>actual)
            return step;
        if(demand==actual)
        return 0;
    }
    static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw)
    {
        geometry_msgs::Quaternion q;
        double t0 = cos(yaw * 0.5);
        double t1 = sin(yaw * 0.5);
        double t2 = cos(roll * 0.5);
        double t3 = sin(roll * 0.5);
        double t4 = cos(pitch * 0.5);
        double t5 = sin(pitch * 0.5);
        q.w = t0 * t2 * t4 + t1 * t3 * t5;
        q.x = t0 * t3 * t4 - t1 * t2 * t5;
        q.y = t0 * t2 * t5 + t1 * t3 * t4;
        q.z = t1 * t2 * t4 - t0 * t3 * t5;
        return q;
    }
    struct EulerAngles
    {
        double roll, pitch, yaw;
    };

    EulerAngles toEulerAngles(geometry_msgs::Quaternion q)
    {
        EulerAngles angles;

        // roll (x-axis rotation)
        double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        angles.roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = +2.0 * (q.w * q.y - q.z * q.x);
        if (fabs(sinp) >= 1)
            angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.pitch = asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        angles.yaw = atan2(siny_cosp, cosy_cosp);

        return angles;
    }
    int i = 0;
    void OnUpdate()
    {

        EulerAngles angles;
        if (modelStates.pose.size() != 0)
        {
            angles = toEulerAngles(modelStates.pose[1].orientation); //Quaternion zwrotny (pobrany z gazebo)

            df = cmd.linear.z; // skręt kół
            double beta_angle = atan(lr / (lr + lf) * tan(df));
            double v = cmd.angular.z;
            double psi = v * sin(beta_angle) / lr;
            double vx = v * cos(angles.yaw);
            double vy = v * sin(angles.yaw);
            this->model->SetLinearVel(ignition::math::Vector3d(vx, vy, 0));
            this->model->SetAngularVel(ignition::math::Vector3d(0, 0, angle_changer(angles.yaw, cmd.linear.z)));
        }
    }

    void pilotCallback(const geometry_msgs::Twist &msg)
    {
        this->cmd = msg;
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates &msg)
    {
        this->modelStates = msg;
    }

private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sub1;
    geometry_msgs::Twist cmd;
    gazebo_msgs::ModelStates modelStates;
    double lf = 10; //podajemy
    double lr = 10; //podajemy
    double df = 0;
};

GZ_REGISTER_MODEL_PLUGIN(ModelPush);
} // namespace gazebo
