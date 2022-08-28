#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float64
from std_msgs.msg import String

def talker():
    pub_voltage = rospy.Publisher('voltage', Float64, queue_size=10)
    pub_temp_outside = rospy.Publisher('temp_outside', Float64, queue_size=10)
    pub_orientation = rospy.Publisher('orientation', String, queue_size=10)

    rospy.init_node('talker_arduino_node')
    #rate = rospy.Rate(10) # 10hz

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    rospy.loginfo("Starting publishing voltage")
    rospy.loginfo("Starting publishing temperature outside")
    rospy.loginfo("Starting publishing orientation")

    while not rospy.is_shutdown():
        dataFromArduino = ser.readline().decode('utf-8').rstrip()
        [dataFromArduino_topic, dataFromArduino_value] = dataFromArduino.split("=")

        if (dataFromArduino_topic == "Voltage"):
            pub_voltage.publish(float(dataFromArduino_value))
            rospy.loginfo(dataFromArduino_topic + "=" + dataFromArduino_value)
        elif (dataFromArduino_topic == "Orientation"):
            [roll, pitch, yaw] = dataFromArduino_value.split(";")
            pub_orientation.publish(roll + "," + pitch + "," + yaw)
            rospy.loginfo(dataFromArduino_topic + "=" + roll + ", " + pitch + ", " + yaw)
        elif (dataFromArduino_topic == "Temperature"):
            pub_temp_outside.publish(float(dataFromArduino_value))
            rospy.loginfo(dataFromArduino_topic + "=" + dataFromArduino_value)
        
        #rate.sleep()

if __name__ == '__main__':
       
    try:
         talker()
    except rospy.ROSInterruptException:
         pass
