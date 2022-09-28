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

global current_velocity_L, current_velocity_R
current_velocity_L = 0
current_velocity_R = 0

global desiredVelocity, desiredVelocity_L, desiredVelocity_R
desiredVelocity = 0
desiredVelocity_L = 0
desiredVelocity_R = 0

pub_setVelocityPWM = rospy.Publisher('/motor/CmdSetVelocityPWM', Int16MultiArray, queue_size=10)
newPWM_array = Int16MultiArray()
newPWM_array.data = []

pub_setVelocity = rospy.Publisher('/motor/CmdSetVelocity', Float32, queue_size=10)

global event, distanceDriven
event = "Empty"
distanceDriven = 0


odom_pub = rospy.Publisher("/motor/odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

## SUBSCRIBERS
def sub_encoderTicks():
    rospy.Subscriber('/motor/encoderTicks', Int16MultiArray, callback_getEncoderTicks)

    global previous_time
    previous_time = rospy.Time.now()

def sub_setVelocity():
    rospy.Subscriber('/motor/CmdSetVelocity', Float32, callback_setVelocity)

def sub_setTurnRadius():
    rospy.Subscriber('/motor/CmdSetTurnRadius', Float32, callback_setTurnRadius)

def sub_joystick():
    rospy.Subscriber("/joystick", String, callback_getJoystickValues)

def sub_resetOdom():
    rospy.Subscriber("/motor/CmdResetOdom", Empty, callback_resetOdom)

def sub_setEvent():
    rospy.Subscriber("/motor/CmdSetEvent", String, callback_setEvent)

## CALLBACKS
def callback_getEncoderTicks(data):
    delta_encoderTick_L = data.data[0]
    delta_encoderTick_R = data.data[1]

    #print("Encoder ticks received [L, R]: " + str(delta_encoderTick_L) + ", " + str(delta_encoderTick_R))

    calcOdom(delta_encoderTick_L, delta_encoderTick_R)
    updateVelocity()

def callback_setVelocity(data):
    global desiredVelocity
    desiredVelocity = data.data

    global desiredVelocity_L, desiredVelocity_R
    desiredVelocity_L = desiredVelocity
    desiredVelocity_R = desiredVelocity

    updateVelocity()

def callback_setTurnRadius(data):
    turnRadius = data.data

    global desiredVelocity
    desiredAngularVelocity = desiredVelocity / turnRadius

    global desiredVelocity_L, desiredVelocity_R
    desiredVelocity_L = (turnRadius + distanceBetweenWheels/2) * desiredAngularVelocity
    desiredVelocity_R = (turnRadius - distanceBetweenWheels/2) * desiredAngularVelocity

    # https://en.wikipedia.org/wiki/Differential_wheeled_robot

    updateVelocity()

def callback_getJoystickValues(data):
    try:
        [key, keyValue] = data.data.split(": ")
    except:
        key = data.data.split(": ")
        keyValue = 0

    keyValue = int(float(keyValue)*255)

    global desiredVelocity_L, desiredVelocity_R
    if (key == "ry"):
        desiredVelocity_L = keyValue
        desiredVelocity_R = keyValue
    elif (key == "rx"):
        desiredVelocity_L = keyValue
        desiredVelocity_R = -keyValue

    updateVelocity()

def callback_resetOdom(Empty):
    rospy.loginfo("Resetting the odometry")

    global x, y, theta
    x = 0
    y = 0
    theta = 0

    current_velocity_x = 0
    current_velocity_y = 0
    current_velocity_theta = 0

    updateOdom(x, y, theta, current_velocity_x, current_velocity_y, current_velocity_theta, rospy.Time.now())

def callback_setEvent(data):
    global event, eventValue
    [event, eventValue] = data.data.split("=")

## FUNCTIONS
def checkEvent():
    global event, eventValue, distanceDriven
    if event == "dist":
        if distanceDriven >= float(eventValue):
            distanceDriven = 0
            pub_setVelocity.publish(0)

def calcOdom(delta_encoderTick_L = 0, delta_encoderTick_R = 0):
    current_time = rospy.Time.now()

    # DISTANCE
    delta_distance_L = distancePerTick * delta_encoderTick_L
    delta_distance_R = distancePerTick * delta_encoderTick_R
    delta_distance = (delta_distance_L + delta_distance_R) / 2

    # VELOCITY
    global current_velocity_L, current_velocity_R, previous_time
    delta_time_sec = (current_time - previous_time).to_sec()
    
    current_velocity_L = delta_distance_L / delta_time_sec
    current_velocity_R = delta_distance_R / delta_time_sec
    delta_velocity = (current_velocity_L + current_velocity_R) / 2

    current_velocity_x = delta_velocity
    current_velocity_y = 0
    current_velocity_theta = ((current_velocity_R - current_velocity_L) / distanceBetweenWheels)

    # ODOMETRY
    global x, y, theta
    delta_theta = (delta_distance_R - delta_distance_L) / (distanceBetweenWheels)
    delta_x = delta_distance * math.cos(theta + (delta_theta / 2))
    delta_y = delta_distance * math.sin(theta + (delta_theta / 2))

    x = x + delta_x
    y = y + delta_y
    theta = theta + delta_theta

    global distanceDriven
    distanceDriven = distanceDriven + math.sqrt(delta_x*delta_x + delta_y*delta_y)

    checkEvent()

    updateOdom(x, y, theta, current_velocity_x, current_velocity_y, current_velocity_theta, current_time)
    
    previous_time = current_time

def updateOdom(x, y, theta, current_velocity_x, current_velocity_y, current_velocity_theta, current_time):
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
    odom.twist.twist = Twist(Vector3(current_velocity_x, current_velocity_y, 0), Vector3(0, 0, current_velocity_theta))

    # publish the message
    odom_pub.publish(odom)

def updateVelocity():
    pid_P = 2
    pid_I = 0
    pid_D = 0.01
    pid_Limits = (maxRPM/60)*distancePerRevolution
    
    global desiredVelocity_L
    pid_L = PID(pid_P, pid_I, pid_D)
    pid_L.setpoint = desiredVelocity_L
    pid_L.output_limits = (-pid_Limits, pid_Limits)
    #pid_L.sample_time = 0.001

    global desiredVelocity_R
    pid_R = PID(pid_P, pid_I, pid_D)
    pid_R.setpoint = desiredVelocity_R
    pid_R.output_limits = (-pid_Limits, pid_Limits)
    #pid_R.sample_time = 0.001

    global current_velocity_L, current_velocity_R
    newVelocity_L = pid_L(current_velocity_L)
    newVelocity_R = pid_R(current_velocity_R)

    #Convert new velocity [m/s] to RPM
    newRPM_L = (newVelocity_L / distancePerRevolution) * 60
    newRPM_R = (newVelocity_R / distancePerRevolution) * 60

    #Convert RPM to PWM-values
    newPWM_L = int((255/maxRPM) * newRPM_L)
    newPWM_R = int((255/maxRPM) * newRPM_R)

    newPWM_array.data = [newPWM_L, newPWM_R]

    pub_setVelocityPWM.publish(newPWM_array)

if __name__ == '__main__':
    rospy.init_node('node_motor', anonymous=True)

    sub_encoderTicks()
    sub_setVelocity()
    sub_setTurnRadius()
    sub_joystick()
    sub_resetOdom()
    sub_setEvent()

    rospy.spin()

