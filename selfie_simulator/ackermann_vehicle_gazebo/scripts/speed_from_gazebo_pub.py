#!/usr/bin/env python
import roslib
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from math import sqrt

def handle_model_pose(msg):
    speed_pub = rospy.Publisher("/speed", Float64, queue_size=100)
    speed_pub.publish(sqrt(msg.twist[1].linear.x*msg.twist[1].linear.x
                     +msg.twist[1].linear.y*msg.twist[1].linear.y))


while not rospy.is_shutdown():
    rospy.init_node("speed_from_gazebo_pub")
    rospy.Subscriber("/gazebo/model_states",
                     ModelStates,
                     handle_model_pose)
    rospy.spin()
