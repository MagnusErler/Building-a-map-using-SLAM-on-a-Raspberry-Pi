#!/usr/bin/env python

# ROS
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32

# Other
import math
from simple_pid import PID

from constants import *

newPWM_array = Int16MultiArray()
newPWM_array.data = []

global previous_encoderTick_L, previous_encoderTick_R
previous_encoderTick_L = 0
previous_encoderTick_R = 0

global delta_speed_L, delta_speed_R
delta_speed_L = 0
delta_speed_R = 0

pub_setSpeedPWM = rospy.Publisher('motor/CmdSetSpeedPWM', Int16MultiArray, queue_size=10)

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

def sub_encoderTicks():
    rospy.init_node('node_motor', anonymous=True)
    rospy.Subscriber('/motor/encoderTicks', Int16MultiArray, callback_getEncoderTicks)

    global previous_time
    current_time = rospy.Time.now()
    previous_time = rospy.Time.now()

def sub_desiredSpeed():
    rospy.Subscriber('/motor/CmdSetSpeed', Float32, callback_setDesiredSpeed)

def callback_getEncoderTicks(data):
    global encoderTick_L, encoderTick_R
    encoderTick_L = data.data[0]
    encoderTick_R = data.data[1]

    #print("Encoder ticks received [L, R]: " + str(encoderTick_L) + ", " + str(encoderTick_R))

    calcOdom()

def callback_setDesiredSpeed(data):
    updateSpeed(data.data)

def calcOdom():
    current_time = rospy.Time.now()

    global previous_encoderTick_L, previous_encoderTick_R, previous_time
    delta_encoderTick_L = encoderTick_L - previous_encoderTick_L
    delta_encoderTick_R = encoderTick_R - previous_encoderTick_R

    delta_time_sec = (current_time - previous_time).to_sec()

    # DISTANCE
    delta_distance_L = distancePerTick * delta_encoderTick_L
    delta_distance_R = distancePerTick * delta_encoderTick_R
    delta_distance = (delta_distance_L + delta_distance_R) / 2

    # SPEED
    global delta_speed_L, delta_speed_R
    delta_speed_L = delta_distance_L / delta_time_sec
    delta_speed_R = delta_distance_R / delta_time_sec
    delta_speed = (delta_speed_L + delta_speed_R) / 2

    delta_speed_x = delta_speed
    delta_speed_y = 0
    delta_speed_theta = ((delta_speed_R - delta_speed_L) / distanceBetweenWheels)

    # ODOMETRY
    global x, y, theta
    delta_theta = (delta_distance_R - delta_distance_L) / (distanceBetweenWheels)
    delta_x = delta_distance * math.cos(theta + (delta_theta / 2))
    delta_y = delta_distance * math.sin(theta + (delta_theta / 2))

    theta = theta + delta_theta
    x = x + delta_x
    y = y + delta_y

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(delta_speed_x, delta_speed_y, 0), Vector3(0, 0, delta_speed_theta))

    # publish the message
    odom_pub.publish(odom)

    previous_encoderTick_L = encoderTick_L
    previous_encoderTick_L = encoderTick_R
    previous_time = current_time

def updateSpeed(desiredSpeed):

    desiredSpeed_L = desiredSpeed
    desiredSpeed_R = desiredSpeed
    
    pid_P = 0.5
    pid_I = 0
    pid_D = 0.01
    
    pid_L = PID(pid_P, pid_I, pid_D, setpoint = desiredSpeed_L)
    pid_L.output_limits = (-5, 5)
    pid_L.sample_time = 0.001

    pid_R = PID(pid_P, pid_I, pid_D, setpoint = desiredSpeed_R)
    pid_R.output_limits = (-5, 5)
    pid_R.sample_time = 0.001

    global delta_speed_L, delta_speed_R
    newSpeed_L = pid_L(delta_speed_L)
    newSpeed_R = pid_R(delta_speed_R)

    #Convert new speed [m/s] to RPM
    newRPM_L = (newSpeed_L / distancePerRevolution) * 60
    newRPM_R = (newSpeed_R / distancePerRevolution) * 60

    #Convert RPM to PWM-values
    newPWM_L = int((255/maxRPM) * newRPM_L)
    newPWM_R = int((255/maxRPM) * newRPM_R)

    newPWM_array.data = [newPWM_L, newPWM_R]
    pub_setSpeedPWM.publish(newPWM_array)

if __name__ == '__main__':
    sub_encoderTicks()
    sub_desiredSpeed()

    rospy.spin()

