import rospy
from std_msgs.msg import Float64MultiArray
from pynput import keyboard

def controller():
    pub = rospy.Publisher('model_control', Float64MultiArray, queue_size=1000);
    rospy.init_node("keyboard_control")
    pygame.init()
    control = [0]*2 #delta, torque
    while not rospy.is_shutdown():
        # acceleration
        if keyboard.is_pressed("w"):
            control[0] = 150
        elif keyboard.is_pressed("s"):
            control[0] = -150
        # steer
        if keyboard.is_pressed("a"):
            control[1] = 0.15
        elif keyboard.is_pressed("d"):
            control[1] = -0.15

        msg = Float64MultiArray(data=control)
        pub.publish(msg)

