#!/usr/bin/env python

import rospy
from subact_gripper.msg import sub_gripper
from std_msgs.msg import Bool
import serial


# ESTE NODO CONECTA EL PUERTO SERIE CON UN TOPIC ROS PARA LA PUBLICACION DE LOS
# DATOS LEIDOS POR EL ARDUINO. SE GRABAN LOS TOPICS NECESARIOS CON ROSBAG

ser = serial.Serial()
ser.baudrate = 500000
ser.port = '/dev/ttyUSB0' # EL PUERTO SE TIENE QUE REVISAR
ser.timeout=1

def control_callback(msg):
    if msg.data:
        ser.write(b'c')
    else:
        ser.write(b'a')


def ardPublisher():
    global ser

    topic = "/robot_subactuated_gripper/gripper_state"
    tctrl = "/robot_subactuated_gripper/close"

    rospy.init_node('serial_sync', anonymous=True)
    pub = rospy.Publisher(topic, sub_gripper, queue_size=1)
    subctrl = rospy.Subscriber("/robot_subactuated_gripper/close",
                                 Bool, control_callback)
    #rate = rospy.Rate(330) # 330 Hz -- cada 0.3 ms

    men = sub_gripper()
    ser.open()
    while not rospy.is_shutdown():
        #with serial.Serial(port, baudrate, timeout=1) as ser:
        try:
            data = str(ser.readline())
        except:
            pass

        #process data.
        data = data.replace("b","").replace("'","").replace("\\r\\n","")#.replace("\\t"," ")
        data = data.split("\\t")#split(" ")
        data.pop()
        data = [float(i) for i in data]
        #print(len(data))
        #create msg
        #print(rospy.get_rostime())
        if len(data) == 12:
            men.dedo_1[0] = data[8]
            men.dedo_1[1] = data[1]
            men.dedo_1[2] = data[0]
            men.dedo_2[0] = data[9]
            men.dedo_2[1] = data[3]
            men.dedo_2[2] = data[2]
            men.dedo_3[0] = data[10]
            men.dedo_3[1] = data[5]
            men.dedo_3[2] = data[4]
            men.dedo_4[0] = data[11]
            men.dedo_4[1] = data[7]
            men.dedo_4[2] = data[6]
            men.header.stamp = rospy.Time.now()
            men.header.frame_id = "Garra_Subact"
            #print(men)
            pub.publish(men)
        #except:
        #    pass


if __name__ == '__main__':
    try:
        ardPublisher()
    except rospy.ROSInterruptException:
        if ser.isOpen():
            ser.close()
