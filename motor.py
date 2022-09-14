#!/usr/bin/env python
#ROS
import rospy
from std_msgs.msg import Int16MultiArray

motorGear = 1/10
wheelDiameter = 0.1
distancePerTick = (3.14159265 * wheelDiameter)

current_time = rospy.Time.now()
previous_time = rospy.Time.now()

def sub_encoderValue():
    rospy.init_node('node_motor', anonymous=True)
    rospy.Subscriber('/motor/encoderTicks', Int16MultiArray, callback_getEncoderTicks)
    rospy.spin()

def callback_getEncoderTicks(data):
    global receivedEncoderValues, encoderValue_L, encoderValue_R

    encoderValue_L = data.data[0]
    encoderValue_R = data.data[1]

    print("Received [L, R]: " + str(encoderValue_L) + ", " + str(encoderValue_R))

if __name__ == '__main__':
    print("Running")
    sub_encoderValue()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        delta_encoderValue_L = encoderValue_L - previous_encoderValue_L
        delta_encoderValue_R = encoderValue_R - previous_encoderValue_R

        delta_time_sec = (current_time - previous_time).toSec()



        previous_time = current_time