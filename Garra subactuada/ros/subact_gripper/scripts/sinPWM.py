#!/usr/bin/env python

import rospy
import math
from subact_gripper.msg import pwm_control

def sinPWM():
    topic = "/robot_subactuated_gripper/pwm"

    rospy.init_node('sinPWM', anonymous=True)
    pub = rospy.Publisher(topic, pwm_control, queue_size=1)
    rate = rospy.Rate(8) # 1 Hz -- cada 0.125 s
    sig = 0
    t = 0

    men = pwm_control()
    while not rospy.is_shutdown():
        #crear mensaje
        men.c1 = sig
        men.c2 = sig
        men.c3 = sig
        men.c4 = sig

        sig = 250+round(200*math.sin(1.5708*t))

        t = t + 0.125
        #send message
        pub.publish(men)

        rate.sleep()



if __name__ == '__main__':
    try:
        sinPWM()
    except rospy.ROSInterruptException:
        pass
