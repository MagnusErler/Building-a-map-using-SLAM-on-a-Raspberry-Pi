#!/usr/bin/env python
#ROS
import rospy
from std_msgs.msg import Int16MultiArray

import math 

motorGear = 1/10
wheelRadius = 0.04  # [m]
distancePerTick = motorGear * (2 * math.pi * wheelRadius)    # [m]
distanceBetweenWheels = 0.24 # [m]

global previous_encoderValue_L, previous_encoderValue_R
previous_encoderValue_L = 0
previous_encoderValue_R = 0

global x, y, theta
x = 0
y = 0
theta = 0

def sub_encoderValue():
    rospy.init_node('node_motor', anonymous=True)
    rospy.Subscriber('/motor/encoderTicks', Int16MultiArray, callback_getEncoderTicks)

    global previous_time
    current_time = rospy.Time.now()
    previous_time = rospy.Time.now()

    rospy.spin()

def callback_getEncoderTicks(data):
    global encoderValue_L, encoderValue_R

    encoderValue_L = data.data[0]
    encoderValue_R = data.data[1]

    print("Received [L, R]: " + str(encoderValue_L) + ", " + str(encoderValue_R))



    current_time = rospy.Time.now()

    global previous_encoderValue_L, previous_encoderValue_R, previous_time
    delta_encoderValue_L = encoderValue_L - previous_encoderValue_L
    delta_encoderValue_R = encoderValue_R - previous_encoderValue_R

    delta_time_sec = (current_time - previous_time)#.toSec()

    # DISTANCE
    delta_distance_L = distancePerTick * delta_encoderValue_L
    delta_distance_R = distancePerTick * delta_encoderValue_R
    delta_distance = (delta_distance_L + delta_distance_R) / 2

    # SPEED
    #delta_speed_L = delta_distance_L / delta_time_sec
    #delta_speed_R = delta_distance_R / delta_time_sec

    # ODOMETRY
    global x, y, theta
    delta_theta = (delta_distance_R - delta_distance_L) / (distanceBetweenWheels)
    delta_x = delta_distance * math.cos(theta + (delta_theta / 2))
    delta_y = delta_distance * math.sin(theta + (delta_theta / 2))

    print("delta_theta: " + str(delta_theta))
    #print("delta_x: " + str(delta_x))
    #print("delta_y: " + str(delta_y))

    theta = theta + delta_theta
    x = x + delta_x
    y = y + delta_y

    #print("x: " + str(x))
    #print("y: " + str(y))
    #print("theta: " + str(theta))


    previous_encoderValue_L = encoderValue_L
    previous_encoderValue_L = encoderValue_R
    previous_time = current_time


if __name__ == '__main__':
    sub_encoderValue()

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        global encoderValue_L, encoderValue_R
        delta_encoderValue_L = encoderValue_L - previous_encoderValue_L
        delta_encoderValue_R = encoderValue_R - previous_encoderValue_R

        delta_time_sec = (current_time - previous_time).toSec()

        # DISTANCE
        delta_distance_L = distancePerTick * delta_encoderValue_L
        delta_distance_R = distancePerTick * delta_encoderValue_R
        delta_distance = (delta_distance_L + delta_distance_R) / 2

        # SPEED
        delta_speed_L = delta_distance_L / delta_time_sec
        delta_speed_R = delta_distance_R / delta_time_sec

        # ODOMETRY
        delta_theta = (delta_distance_R - delta_distance_L) / (distanceBetweenWheels)
        delta_x = delta_distance * cos(theta + (delta_theta / 2))
        delta_y = delta_distance * sin(theta + (delta_theta / 2))

        theta = theta + delta_theta
        x = x + delta_x
        y = y + delta_y

        print("x: " + str(x))
        print("y: " + str(y))
        print("theta: " + str(theta))


        previous_encoderValue_L = encoderValue_L
        previous_encoderValue_L = encoderValue_R
        previous_time = current_time
