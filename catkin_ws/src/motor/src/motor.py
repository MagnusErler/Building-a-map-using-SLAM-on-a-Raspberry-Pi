#!/usr/bin/env python

# ROS
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String

# Other
import sys
import math
from simple_pid import PID

sys.path.insert(1, '/home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi')
from constants import *

global previous_encoderTick_L, previous_encoderTick_R
previous_encoderTick_L = 0
previous_encoderTick_R = 0

global current_speed_L, current_speed_R
current_speed_L = 0
current_speed_R = 0

global desiredSpeed_L, desiredSpeed_R
desiredSpeed_L = 0
desiredSpeed_R = 0

usingJoystick = False

pub_setSpeedPWM = rospy.Publisher('motor/CmdSetSpeedPWM', Int16MultiArray, queue_size=10)
newPWM_array = Int16MultiArray()
newPWM_array.data = []

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

def sub_encoderTicks():
    rospy.Subscriber('/motor/encoderTicks', Int16MultiArray, callback_getEncoderTicks)

    global previous_time
    previous_time = rospy.Time.now()

def sub_desiredSpeed():
    rospy.Subscriber('/motor/CmdSetSpeed', Float32, callback_setDesiredSpeed)

def sub_joystick():
    rospy.Subscriber("/joystick", String, callback_getJoystickValues)

def sub_resetOdom():
    rospy.Subscriber("/motor/CmdResetOdom", Empty, callback_resetOdom)

def callback_getEncoderTicks(data):
    delta_encoderTick_L = data.data[0]
    delta_encoderTick_R = data.data[1]

    #print("Encoder ticks received [L, R]: " + str(delta_encoderTick_L) + ", " + str(delta_encoderTick_R))

    calcOdom(delta_encoderTick_L, delta_encoderTick_R)
    updateSpeed()

def callback_setDesiredSpeed(data):
    desiredSpeed = data.data

    global desiredSpeed_L, desiredSpeed_R
    desiredSpeed_L = desiredSpeed
    desiredSpeed_R = desiredSpeed

    updateSpeed()

def callback_getJoystickValues(data):
    try:
        [key, value] = data.data.split(": ")
    except:
        key = data.data.split(": ")
        value = 0

    value = int(float(value)*255)

    global desiredSpeed_L, desiredSpeed_R
    if (key == "ry"):
        desiredSpeed_L = value
        desiredSpeed_R = value
    elif (key == "rx"):
        desiredSpeed_L = value
        desiredSpeed_R = -value

    updateSpeed()

def callback_resetOdom(Empty):
    rospy.loginfo("Resetting the odometry")

    global x, y, theta
    x = 0
    y = 0
    theta = 0

    current_speed_x = 0
    current_speed_y = 0
    current_speed_theta = 0

    updateOdom(x, y, theta, current_speed_x, current_speed_y, current_speed_theta, rospy.Time.now())

def calcOdom(delta_encoderTick_L = 0, delta_encoderTick_R = 0):
    current_time = rospy.Time.now()

    # DISTANCE
    delta_distance_L = distancePerTick * delta_encoderTick_L
    delta_distance_R = distancePerTick * delta_encoderTick_R
    delta_distance = (delta_distance_L + delta_distance_R) / 2

    # SPEED
    global current_speed_L, current_speed_R, previous_time
    delta_time_sec = (current_time - previous_time).to_sec()
    
    current_speed_L = delta_distance_L / delta_time_sec
    current_speed_R = delta_distance_R / delta_time_sec
    delta_speed = (current_speed_L + current_speed_R) / 2

    current_speed_x = delta_speed
    current_speed_y = 0
    current_speed_theta = ((current_speed_R - current_speed_L) / distanceBetweenWheels)

    # ODOMETRY
    global x, y, theta
    delta_theta = (delta_distance_R - delta_distance_L) / (distanceBetweenWheels)
    delta_x = delta_distance * math.cos(theta + (delta_theta / 2))
    delta_y = delta_distance * math.sin(theta + (delta_theta / 2))

    x = x + delta_x
    y = y + delta_y
    theta = theta + delta_theta

    updateOdom(x, y, theta, current_speed_x, current_speed_y, current_speed_theta, current_time)
    
    previous_time = current_time

def updateOdom(x, y, theta, current_speed_x, current_speed_y, current_speed_theta, current_time):
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
    odom.twist.twist = Twist(Vector3(current_speed_x, current_speed_y, 0), Vector3(0, 0, current_speed_theta))

    # publish the message
    odom_pub.publish(odom)

def updateSpeed():    
    pid_P = 2
    pid_I = 0
    pid_D = 0.01
    pid_Limits = (maxRPM/60)*distancePerRevolution
    
    global desiredSpeed_L
    pid_L = PID(pid_P, pid_I, pid_D)
    pid_L.setpoint = desiredSpeed_L
    pid_L.output_limits = (-pid_Limits, pid_Limits)
    #pid_L.sample_time = 0.001

    global desiredSpeed_R
    pid_R = PID(pid_P, pid_I, pid_D)
    pid_R.setpoint = desiredSpeed_R
    pid_R.output_limits = (-pid_Limits, pid_Limits)
    #pid_R.sample_time = 0.001

    global current_speed_L, current_speed_R
    newSpeed_L = pid_L(current_speed_L)
    newSpeed_R = pid_R(current_speed_R)

    #Convert new speed [m/s] to RPM
    newRPM_L = (newSpeed_L / distancePerRevolution) * 60
    newRPM_R = (newSpeed_R / distancePerRevolution) * 60

    #Convert RPM to PWM-values
    newPWM_L = int((255/maxRPM) * newRPM_L)
    newPWM_R = int((255/maxRPM) * newRPM_R)

    newPWM_array.data = [newPWM_L, newPWM_R]

    pub_setSpeedPWM.publish(newPWM_array)

if __name__ == '__main__':
    rospy.init_node('node_motor', anonymous=True)

    sub_encoderTicks()
    sub_desiredSpeed()
    sub_joystick()
    sub_resetOdom()

    rospy.spin()

