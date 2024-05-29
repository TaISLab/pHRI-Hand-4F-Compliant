#!/usr/bin/env python3

import rospy
from subact_gripper.msg import force_meas_dev
import serial


# Programa de publicacion de los datos del dispositivo de medida de la fuerza de agarre (grasping force -- gforce)

ser = serial.Serial()
ser.baudrate = 250000
ser.port = '/dev/ttyACM0' # EL PUERTO SE TIENE QUE REVISAR
ser.timeout=1


def ardPublisher():
    global ser

    topic = "/robot_subactuated_gripper/ground_truth_grasping_force"

    rospy.init_node('force_meassuring_dev', anonymous=True)
    pub = rospy.Publisher(topic, force_meas_dev, queue_size=1)
    #rate = rospy.Rate(330) # 330 Hz -- cada 0.3 ms

    men = force_meas_dev()
    ser.open()
    while not rospy.is_shutdown():
        #with serial.Serial(port, baudrate, timeout=1) as ser:
        try:
            data = str(ser.readline())
        except:
            pass

        #process data.
        data = data.replace("b","").replace("'","").replace("\\r\\n","")#.replace("\\t"," ")
        data = data.split(" ")#split(" ")

        #data.pop()
        try:
            data = [float(i) for i in data]
            #print(len(data))
            #create msg
            #print(rospy.get_rostime())
            if len(data) == 2:
                men.sensor1 = data[0]
                men.sensor2 = data[1]
                men.header.stamp = rospy.Time.now()
                men.header.frame_id = "Gforce_Meas_Dev"
                #print(men)
                pub.publish(men)
        except:
            pass


if __name__ == '__main__':
    try:
        ardPublisher()
    except rospy.ROSInterruptException:
        if ser.isOpen():
            ser.close()
