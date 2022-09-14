#!/usr/bin/env python
#ROS
import rospy
from std_msgs.msg import Int16MultiArray

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