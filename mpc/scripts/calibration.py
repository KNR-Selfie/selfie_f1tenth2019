#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Float32

sum = 0
counter = 0

def handle_speed(msg):
    global sum, counter
    sum += speed.data
    counter += 1

    if counter == 100:
        print(sum/counter)
        sum = 0
        counter = 0

while not rospy.is_shutdown():
    rospy.init_node("calibration")
    rospy.Subscriber("/speed", Float32, handle_speed)
    rospy.spin()
