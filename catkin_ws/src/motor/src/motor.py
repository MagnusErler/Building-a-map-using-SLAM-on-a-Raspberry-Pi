#!/usr/bin/env python

# ROS
import rospy
import tf
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int16MultiArray, Float32, String

# Other
import sys
import math
from simple_pid import PID

sys.path.insert(1, '/home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi')
from constants import *

global delta_encoderTick_L, delta_encoderTick_R
delta_encoderTick_L = 0
delta_encoderTick_R = 0

# POSITION
global currentPosition_x, currentPosition_y
currentPosition_x = 0   # [m]
currentPosition_y = 0   # [m]

# ORIENTATION
global roll, pitch, yaw, currentOrientation_theta
roll = 0    # [degrees]
pitch = 0   # [degrees]
yaw = 0     # [degrees]
currentOrientation_theta = 0    # [rad]

global quaternion
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

# VELOCITY
global currentVelocity_L, currentVelocity_R
currentVelocity_L = 0   # [m/s]
currentVelocity_R = 0   # [m/s]

global desiredVelocity, desiredVelocity_L, desiredVelocity_R
desiredVelocity = 0     # [m/s]
desiredVelocity_L = 0   # [m/s]
desiredVelocity_R = 0   # [m/s]

newPWM_array = Int16MultiArray()
newPWM_array.data = []

# ODOMETRY
odom_broadcaster = tf.TransformBroadcaster()
odom = Odometry()

# SETUP PUBLISHERS
pub_odom = rospy.Publisher("/motor/odom", Odometry, queue_size=50)
pub_setVelocityPWM = rospy.Publisher('/motor/CmdSetVelocityPWM', Int16MultiArray, queue_size=10)
pub_setVelocity = rospy.Publisher('/motor/CmdSetVelocity', Float32, queue_size=10)

# EVENT
global event, distanceDriven
event = "Empty"
distanceDriven = 0

def setupSubscribers():
    rospy.Subscriber("/IMU/orientation", Int16MultiArray, callback_getOrientation)
    rospy.Subscriber("/joystick", String, callback_getJoystickValues)
    rospy.Subscriber("/motor/CmdResetOdom", Empty, callback_resetOdom)
    rospy.Subscriber("/motor/CmdSetEvent", String, callback_setEvent)
    rospy.Subscriber('/motor/CmdSetVelocity', Float32, callback_setVelocity)
    rospy.Subscriber('/motor/CmdSetTurnRadius', Float32, callback_setTurnRadius)
    rospy.Subscriber('/motor/encoderTicks', Int16MultiArray, callback_getEncoderTicks)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_getPoseGoal)

## CALLBACKS
def callback_getOrientation(data):
    global roll, pitch, yaw
    roll = data.data[1]     # [degrees]
    pitch = data.data[0]    # [degrees]
    yaw = data.data[2]      # [degrees]

def callback_getJoystickValues(data):
    try:
        [key, keyValue] = data.data.split(": ")
    except:
        key = data.data.split(": ")
        keyValue = 0

    desiredJoystickVelocity = float(keyValue) * maxVelocity # 2.51327 [m/s]
    
    global desiredVelocity_L, desiredVelocity_R
    if (key == "ry"):
        desiredVelocity_L = desiredJoystickVelocity     # [m/s]
        desiredVelocity_R = desiredJoystickVelocity     # [m/s]
    elif (key == "rx"):
        desiredVelocity_L = desiredJoystickVelocity     # [m/s]
        desiredVelocity_R = -desiredJoystickVelocity    # [m/s]

    updateVelocity()

def callback_resetOdom(Empty):
    rospy.loginfo("Resetting the odometry")

    global currentPosition_x, currentPosition_y, currentOrientation_theta
    currentPosition_x = 0
    currentPosition_y = 0
    currentOrientation_theta = 0

    current_velocity_x = 0
    current_velocity_y = 0
    current_velocity_theta = 0

    updateOdom(currentPosition_x, currentPosition_y, currentOrientation_theta, current_velocity_x, current_velocity_y, current_velocity_theta, rospy.Time.now())

def callback_setEvent(data):
    global event, eventValue
    [event, eventValue] = data.data.split("=")

def callback_setVelocity(data):
    global desiredVelocity
    desiredVelocity = data.data # [m/s]

    global desiredVelocity_L, desiredVelocity_R
    desiredVelocity_L = desiredVelocity # [m/s]
    desiredVelocity_R = desiredVelocity # [m/s]

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

def callback_getEncoderTicks(data):
    global delta_encoderTick_L, delta_encoderTick_R
    delta_encoderTick_L = data.data[0]
    delta_encoderTick_R = data.data[1]

    #print("Encoder ticks received [L, R]: " + str(delta_encoderTick_L) + ", " + str(delta_encoderTick_R))

    calcOdom(delta_encoderTick_L, delta_encoderTick_R)
    updateVelocity()

def callback_getPoseGoal(data):
    #rospy.loginfo("Received at goal message!")
    #rospy.loginfo("Timestamp: " + str(data.header.stamp))
    #rospy.loginfo("frame_id: " + str(data.header.frame_id))

    #position = data.pose.position

    desiredPosition_x = data.pose.position.x
    desiredPosition_y = data.pose.position.y

    driveToXYPosition(desiredPosition_x, desiredPosition_y)
    
    quat = data.pose.orientation
    #rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

    euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    #rospy.loginfo("Euler Angles: %s"%str(euler))

    #driveToThetaOrientation(euler[2])

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
    delta_distance_L = distancePerTick * delta_encoderTick_L    # [m]
    delta_distance_R = distancePerTick * delta_encoderTick_R    # [m]
    delta_distance = (delta_distance_L + delta_distance_R) / 2  # [m]

    # VELOCITY
    global currentVelocity_L, currentVelocity_R, previous_time
    delta_time_sec = (current_time - previous_time).to_sec()
    
    currentVelocity_L = delta_distance_L / delta_time_sec          # [m/s]
    currentVelocity_R = delta_distance_R / delta_time_sec          # [m/s]
    delta_velocity = (currentVelocity_L + currentVelocity_R) / 2  # [m/s]

    current_velocity_x = delta_velocity # [m/s]
    current_velocity_y = 0              # [m/s]
    current_velocity_theta = (currentVelocity_R - currentVelocity_L) / distanceBetweenWheels

    # ODOMETRY
    global currentPosition_x, currentPosition_y, currentOrientation_theta
    delta_theta = (delta_distance_R - delta_distance_L) / distanceBetweenWheels
    delta_x = delta_distance * math.cos(currentOrientation_theta + (delta_theta / 2))
    delta_y = delta_distance * math.sin(currentOrientation_theta + (delta_theta / 2))

    # Current position according to odometry
    currentPosition_x = currentPosition_x + delta_x
    currentPosition_y = currentPosition_y + delta_y

    # Current orientation according to odometry
    currentOrientation_theta = currentOrientation_theta + delta_theta

    if currentOrientation_theta > math.pi:
        currentOrientation_theta = currentOrientation_theta - 2*math.pi
    elif currentOrientation_theta < -math.pi:
        currentOrientation_theta = currentOrientation_theta + 2*math.pi

    global distanceDriven
    distanceDriven = distanceDriven + math.sqrt(delta_x*delta_x + delta_y*delta_y)

    checkEvent()

    updateOdom(currentPosition_x, currentPosition_y, currentOrientation_theta, current_velocity_x, current_velocity_y, current_velocity_theta, current_time)
    
    previous_time = current_time

def updateOdom(currentPosition_x, currentPosition_y, currentOrientation_theta, current_velocity_x, current_velocity_y, current_velocity_theta, current_time):
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    global roll, pitch, yaw # [degrees]
    #quaternion = tf.transformations.quaternion_from_euler(0, 0, currentOrientation_theta)
    #quaternion = tf.transformations.quaternion_from_euler(roll*(math.pi/180), pitch*(math.pi/180), yaw*(math.pi/180))
    #quaternion = tf.transformations.quaternion_from_euler(roll*(math.pi/180), pitch*(math.pi/180), currentOrientation_theta)
    global quaternion
    quaternion = tf.transformations.quaternion_from_euler(0, 0, currentOrientation_theta)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (currentPosition_x, currentPosition_y, 0.),
        quaternion,
        current_time,
        "robot",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(currentPosition_x, currentPosition_y, 0.), Quaternion(*quaternion))

    # set the velocity
    odom.child_frame_id = "robot"
    odom.twist.twist = Twist(Vector3(current_velocity_x, current_velocity_y, 0), Vector3(0, 0, current_velocity_theta))

    # publish the message
    pub_odom.publish(odom)

def updateVelocity():
    pid_P = 0.00548
    pid_I = 0.0423
    pid_D = 0.000177

    pid_P = 1
    pid_I = 0
    pid_D = 0

    pid_Limits = maxVelocity  # 2.51327 [m/s]
    
    global desiredVelocity_L    # [m/s]
    pid_L = PID(pid_P, pid_I, pid_D)
    pid_L.setpoint = desiredVelocity_L
    pid_L.output_limits = (-pid_Limits, pid_Limits)
    #pid_L.sample_time = 0.001

    global desiredVelocity_R    # [m/s]
    pid_R = PID(pid_P, pid_I, pid_D)
    pid_R.setpoint = desiredVelocity_R
    pid_R.output_limits = (-pid_Limits, pid_Limits)
    #pid_R.sample_time = 0.001

    global currentVelocity_L, currentVelocity_R
    newVelocity_L = pid_L(currentVelocity_L)   # [m/s]
    newVelocity_R = pid_R(currentVelocity_R)   # [m/s]

    newVelocity_L = newVelocity_L * (100 + motorSpeedOffset)/100    # [m/s]
    newVelocity_R = newVelocity_R * (100 - motorSpeedOffset)/100    # [m/s]

    #Convert new velocity [m/s] to RPM
    newRPM_L = (newVelocity_L / distancePerRevolution) * 60 # [RPM]
    newRPM_R = (newVelocity_R / distancePerRevolution) * 60 # [RPM]

    #Convert RPM to PWM-values
    newPWM_L = int((maxPWM/maxRPM) * newRPM_L)  # [PWM]
    newPWM_R = int((maxPWM/maxRPM) * newRPM_R)  # [PWM]

    newPWM_array.data = [newPWM_L, newPWM_R]

    pub_setVelocityPWM.publish(newPWM_array)

def driveStraight():

    global delta_encoderTick_L, delta_encoderTick_R
    velocity_offset = 0
    if delta_encoderTick_L > delta_encoderTick_R:
        velocity_offset = -0.1  # [m/s]
    elif delta_encoderTick_L < delta_encoderTick_R:
        velocity_offset = 0.1   # [m/s]

    global desiredVelocity_L, desiredVelocity_R
    desiredVelocity_L = desiredVelocity_L + velocity_offset    # [m/s]
    desiredVelocity_R = desiredVelocity_R - velocity_offset    # [m/s]

    print(desiredVelocity_L)
    print(desiredVelocity_R)

    updateVelocity()
    

    """
    global currentOrientation_theta

    driveStraight_desiredOrientation_theta = currentOrientation_theta

    velocity_offset = 0
    if driveStraight_desiredOrientation_theta < currentOrientation_theta:
        velocity_offset = -0.1  # [m/s]
    elif driveStraight_desiredOrientation_theta > currentOrientation_theta:
        velocity_offset = 0.1  # [m/s]

    global desiredVelocity_L, desiredVelocity_R
    desiredVelocity_L = desiredVelocity_L + velocity_offset    # [m/s]
    desiredVelocity_R = desiredVelocity_R - velocity_offset    # [m/s]

    print(desiredVelocity_L)
    print(desiredVelocity_R)

    updateVelocity()
    """

def driveToXYPosition(desiredPosition_x, desiredPosition_y):

    rospy.loginfo("Driving to position: " + str(desiredPosition_x) + ", " + str(desiredPosition_y))

    r = rospy.Rate(20)

    tries = 0

    while tries < 1000:

        global currentPosition_x, currentPosition_y
        distanceToGoal_x = desiredPosition_x - currentPosition_x    # [m]
        distanceToGoal_y = desiredPosition_y - currentPosition_y    # [m]

        distanceToGoal = math.sqrt(distanceToGoal_x*distanceToGoal_x + distanceToGoal_y*distanceToGoal_y)   # [m]
        angleToGoal = math.atan2(distanceToGoal_y, distanceToGoal_x)    # [rad]

        global desiredVelocity_L, desiredVelocity_R

        if distanceToGoal < 0.05:
            rospy.loginfo("DONE: Desired position (" + str(desiredPosition_x) + ", " + str(desiredPosition_y) + ") has been reached")

            desiredVelocity_L = 0   # [m/s]
            desiredVelocity_R = 0   # [m/s]
            updateVelocity()
            return

        global quaternion
        odom_euler = tf.transformations.euler_from_quaternion(quaternion)

        angleFromRobotToGoal = angleToGoal - odom_euler[2]

        if abs(angleFromRobotToGoal) > 0.1:
            desiredVelocity_L = -0.5    # [m/s]
            desiredVelocity_R = 0.5     # [m/s]

            tries = tries + 1
        else:
            desiredVelocity_L = 1   # [m/s]
            desiredVelocity_R = 1   # [m/s]

            tries = 0

        updateVelocity()

        r.sleep()  
    
    print("Could NOT find the goal after 1000 tries")
    desiredVelocity_L = 0   # [m/s]
    desiredVelocity_R = 0   # [m/s]
    updateVelocity()

def driveToThetaOrientation(desiredOrientation_theta):

    rospy.loginfo("Driving to orientation: " + str(desiredOrientation_theta) + " rad")

    r = rospy.Rate(20)

    tries = 0

    while tries < 1000:

        if desiredOrientation_theta < 0.1:
            rospy.loginfo("DONE: Desired orientation (" + str(desiredOrientation_theta) + ") has been reached")

            desiredVelocity_L = 0   # [m/s]
            desiredVelocity_R = 0   # [m/s]
            updateVelocity()
            return

        if abs(desiredOrientation_theta) > 0.5:
            desiredVelocity_L = -0.5    # [m/s]
            desiredVelocity_R = 0.5     # [m/s]

            tries = tries + 1

    print("Could NOT find the goal after 1000 tries")
    desiredVelocity_L = 0   # [m/s]
    desiredVelocity_R = 0   # [m/s]
    updateVelocity()

if __name__ == '__main__':
    rospy.init_node('node_motor', anonymous=True)

    setupSubscribers()

    global previous_time
    previous_time = rospy.Time.now()

    rospy.spin()

