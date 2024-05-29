#!/usr/bin/env python

import rospy
import math
from subact_gripper.msg import pwm_control

def impulsePWM():
    topic = "/robot_subactuated_gripper/pwm"

    rospy.init_node('impulsePWM', anonymous=True)
    pub = rospy.Publisher(topic, pwm_control, queue_size=1)
    rate = rospy.Rate(8) # 1 Hz -- cada 0.125 s
    sig = 300
    t = 0
    reached = 0
    tant = 0

    men = pwm_control()
    while not rospy.is_shutdown():
        t = t + 0.125
        #crear mensaje
        men.c1 = sig
        men.c2 = sig
        men.c3 = sig
        men.c4 = sig

        if t-tant == 2:
            tant = t
            if reached == 0:
                sig = 400
                reached = 1
            else:
                sig = 70
                reached = 0

        #send message
        pub.publish(men)
        rate.sleep()



if __name__ == '__main__':
    try:
        impulsePWM()
    except rospy.ROSInterruptException:
        pass
