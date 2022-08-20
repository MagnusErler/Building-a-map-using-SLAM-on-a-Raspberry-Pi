#!/usr/bin/env python3
# license removed for brevity
import rospy
import serial
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('voltage', String, queue_size=10)
    rospy.init_node('talker_node')
    rate = rospy.Rate(10) # 10hz

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()

    rospy.loginfo("Starting publishing voltage")

    while not rospy.is_shutdown():
        dataFromArduino = ser.readline().decode('utf-8').rstrip()
        [dataFromArduino_topic, dataFromArduino_value] = dataFromArduino.split(":")

        pub.publish(dataFromArduino_value)
        rospy.loginfo(dataFromArduino_topic + ": " + dataFromArduino_value)
        rate.sleep()

if __name__ == '__main__':
        
    try:
         talker()
    except rospy.ROSInterruptException:
         pass
