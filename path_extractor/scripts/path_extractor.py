#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import math
import sys
import pickle
import numpy as np

from std_msgs.msg import Float64
from nav_msgs.msg import Path

UPDATE_RATE = 100

if __name__ == '__main__':
    rospy.init_node('path_extractor')

    #Get Parameters
    points_backwards = rospy.get_param('~path_points_backwards', 2)
    points_forwards = rospy.get_param('~path_points_forwards', 5)
    path_direction = rospy.get_param('~path_direction', 0)
    direction = 0
    #Direction will be reversed by multiplying by -1
    if path_direction == 1:
        direction = -1
    else:
        direction = 1

    #Read input file
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        map_data = pickle.load(f)
    pathpoints = map_data['pathpoints']

    #Topic publishers
    path_pub = rospy.Publisher('closest_path_points', Path, queue_size=UPDATE_RATE)

    #Configure transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    #Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)


    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        try:
            pose = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        x = pose.translation.x
        y = pose.translation.y

        point = (x, y)
        closest_point_index = np.argmin(np.sum(np.square(np.array(point)-pathpoints), 1))
        #Find index of the first point of the published path
        current_index = (closest_point_index + len(pathpoints) - points_backwards*direction) % len(pathpoints)
        last_index = (closest_point_index + points_forwards*direction) % len(pathpoints)

        output_path = Path()
        output_path.header.stamp = current_time
        output_path.header.frame_id = 'map'
        output_path.poses = []
        while not current_index == last_index:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = current_time
            pose.pose.position.x = pathpoints[current_index][0]
            pose.pose.position.y = pathpoints[current_index][1]
            output_path.poses.append(pose)
            current_index += direction
            current_index = (current_index + len(pathpoints)) % len(pathpoints)

        path_pub.publish(output_path)

        rate.sleep()
