#!/usr/bin/env python

import rospy
from subact_gripper.msg import pwm_control

def rampaPWM():
    topic = "/robot_subactuated_gripper/pwm"

    rospy.init_node('rampaPWM', anonymous=True)
    pub = rospy.Publisher(topic, pwm_control, queue_size=1)
    rate = rospy.Rate(8) # 8 Hz -- cada 0.25 s
    sig = 45
    reached = 0
    it = 0
    men = pwm_control()
    while not rospy.is_shutdown():
        #crear mensaje
        men.c1 = sig
        men.c2 = sig
        men.c3 = sig
        men.c4 = sig
        if reached == 0:
            sig = sig + 15
        else:
            sig = sig - 15

        if sig >= 390:
            reached = 1
        elif sig <= 45:
             reached = 0
             #it = it +1

        if it == 4:
            break
        #send message
        pub.publish(men)
        rate.sleep()

    sig = 0
    men.c1 = sig
    men.c2 = sig
    men.c3 = sig
    men.c4 = sig
    pub.publish(men)



if __name__ == '__main__':
    try:
        rampaPWM()
    except rospy.ROSInterruptException:
        pass
