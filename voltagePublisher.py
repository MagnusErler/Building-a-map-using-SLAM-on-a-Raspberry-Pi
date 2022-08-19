#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=5)
    ser.reset_input_buffer()

    while not rospy.is_shutdown():
        voltage = ser.readline().decode('utf-8').rstrip()
        rospy.loginfo(voltage)
        pub.publish(voltage)
        rate.sleep()

if __name__ == '__main__':
        
    try:
         talker()
    except rospy.ROSInterruptException:
         pass
