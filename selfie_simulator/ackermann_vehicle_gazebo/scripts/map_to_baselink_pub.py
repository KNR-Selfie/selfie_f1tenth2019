#!/usr/bin/env python
import roslib
import rospy
import tf
from gazebo_msgs.msg import ModelStates


def handle_model_pose(msg):
    br = tf.TransformBroadcaster()
    print(msg.pose[1].position.x)
    br.sendTransform((msg.pose[1].position.x, msg.pose[1].position.y,
                     msg.pose[1].position.z),
                     (msg.pose[1].orientation.x, msg.pose[1].orientation.y,
                     msg.pose[1].orientation.z, msg.pose[1].orientation.w),
                     rospy.Time.now(),
                     "base_",
                     "map");



while not rospy.is_shutdown():
    rospy.init_node("map_to_baselink_pub")
    rospy.Subscriber("/gazebo/model_states",
                     ModelStates,
                     handle_model_pose)
    rospy.spin()
