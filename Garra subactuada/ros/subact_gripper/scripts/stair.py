#!/usr/bin/env python

import rospy
import math
from subact_gripper.msg import pwm_control

def stairPWM():
    topic = "/robot_subactuated_gripper/pwm"

    rospy.init_node('stairPWM', anonymous=True)
    pub = rospy.Publisher(topic, pwm_control, queue_size=1)
    rate = rospy.Rate(8) # 1 Hz -- cada 0.25 s
    sig = 70
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

        if t-tant == 2 and reached==0:
            tant = t
            if reached == 0:
                sig = sig+100
        elif reached==1:
            sig = sig-2


        if sig == 270:
            reached = 1
        elif sig == 70 and reached==1:
            reached = 0
            tant = t

        #send message
        pub.publish(men)
        rate.sleep()



if __name__ == '__main__':
    try:
        stairPWM()
    except rospy.ROSInterruptException:
        pass
