#!/usr/bin/env python

import numpy as np
from scipy.interpolate import CubicSpline
from std_msgs.msg import Float64MultiArray
import rospy
import tf2_ros as tf2
import tf
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from std_msgs.msg import Float64



MAP_FRAME = 'skidpad'
VEHICLE_FRAME = 'base_link'

PUBLISH_RATE = 40

path = None

def handle_path(msg):
    global path
    path = msg


rospy.init_node('calc_path_coefs')
coef_pub = rospy.Publisher('trajectory_coefs', Float64MultiArray, queue_size=10)
path_sub = rospy.Subscriber('path', Path, handle_path)
listener = tf.TransformListener()

rate = rospy.Rate(PUBLISH_RATE)
d = rospy.Duration(2.0)
rospy.sleep(d)
while not rospy.is_shutdown():

    try:
        listener.waitForTransform(MAP_FRAME,
          VEHICLE_FRAME,
          rospy.Time(0),
          rospy.Duration(4.0))

        x = []
        y = []

        point_map = PointStamped()
        if path != None:
            for pose in path.poses:
                point_map.header.frame_id = MAP_FRAME
                point_map.header.stamp = rospy.Time(0)
                point_map.point = pose.pose.position
                point_vehicle = listener.transformPoint(VEHICLE_FRAME, point_map)
                x.append(point_vehicle.point.x)
                y.append(point_vehicle.point.y)

        coefs = Float64MultiArray()
        coefs.data = np.polyfit(x, y, deg=2)
        strictly_increasing = Trueg
        for i in range(1, len(x)):
            if x[i] <= x[i - 1]:
                strictly_increasing = False
        if strictly_increasing:
            spline = CubicSpline(x, y)
            print(spline)
        coef_pub.publish(coefs)

    except (tf.LookupException, tf.ConnectivityException):
        rospy.logwarn('Transform lookup failed')
    rate.sleep()
