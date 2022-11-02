#!/usr/bin/env python

# ROS
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
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
current_velocity_L = 0  # [m/s]
current_velocity_R = 0  # [m/s]

global desiredVelocity, desiredVelocity_L, desiredVelocity_R
desiredVelocity = 0     # [m/s]
desiredVelocity_L = 0   # [m/s]
desiredVelocity_R = 0   # [m/s]

global desiredPosition_x, desiredPosition_y, desiredPosition_z
desiredPosition_x = 0   # [m]
desiredPosition_y = 0   # [m]

pub_setVelocityPWM = rospy.Publisher('/motor/CmdSetVelocityPWM', Int16MultiArray, queue_size=10)
newPWM_array = Int16MultiArray()
newPWM_array.data = []

pub_setVelocity = rospy.Publisher('/motor/CmdSetVelocity', Float32, queue_size=10)

global event, distanceDriven
event = "Empty"
distanceDriven = 0

global odom_quat
odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

odom_pub = rospy.Publisher("/motor/odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

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
    delta_encoderTick_L = data.data[0]
    delta_encoderTick_R = data.data[1]

    #print("Encoder ticks received [L, R]: " + str(delta_encoderTick_L) + ", " + str(delta_encoderTick_R))

    calcOdom(delta_encoderTick_L, delta_encoderTick_R)
    updateVelocity()

def callback_getPoseGoal(data):
    #rospy.loginfo("Received at goal message!")
    #rospy.loginfo("Timestamp: " + str(data.header.stamp))
    #rospy.loginfo("frame_id: " + str(data.header.frame_id))

    position = data.pose.position

    global desiredPosition_x, desiredPosition_y
    desiredPosition_x = data.pose.position.x
    desiredPosition_y = data.pose.position.y
    
    rospy.loginfo("Point Position: [ %f, %f ]"%(desiredPosition_x, desiredPosition_y))

    drive2Goal()
    
    #quat = data.pose.orientation
    #rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))

    #euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    #rospy.loginfo("Euler Angles: %s"%str(euler))


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
    global current_velocity_L, current_velocity_R, previous_time
    delta_time_sec = (current_time - previous_time).to_sec()
    
    current_velocity_L = delta_distance_L / delta_time_sec          # [m/s]
    current_velocity_R = delta_distance_R / delta_time_sec          # [m/s]
    delta_velocity = (current_velocity_L + current_velocity_R) / 2  # [m/s]

    current_velocity_x = delta_velocity # [m/s]
    current_velocity_y = 0              # [m/s]
    current_velocity_theta = ((current_velocity_R - current_velocity_L) / distanceBetweenWheels)

    # ODOMETRY
    global x, y, theta
    delta_theta = (delta_distance_R - delta_distance_L) / (distanceBetweenWheels)
    delta_x = delta_distance * math.cos(theta + (delta_theta / 2))
    delta_y = delta_distance * math.sin(theta + (delta_theta / 2))

    # Current position according to odometry
    x = x + delta_x
    y = y + delta_y
    theta = theta + delta_theta

    if theta > math.pi:
        theta = theta - 2*math.pi
    elif theta < -math.pi:
        theta = theta + 2*math.pi

    global distanceDriven
    distanceDriven = distanceDriven + math.sqrt(delta_x*delta_x + delta_y*delta_y)

    checkEvent()

    updateOdom(x, y, theta, current_velocity_x, current_velocity_y, current_velocity_theta, current_time)
    
    previous_time = current_time

def updateOdom(x, y, theta, current_velocity_x, current_velocity_y, current_velocity_theta, current_time):
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    global roll, pitch, yaw# [degrees]
    #odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    #odom_quat = tf.transformations.quaternion_from_euler(roll*(math.pi/180), pitch*(math.pi/180), yaw*(math.pi/180))
    #odom_quat = tf.transformations.quaternion_from_euler(roll*(math.pi/180), pitch*(math.pi/180), theta)
    global odom_quat
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "robot",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "robot"
    odom.twist.twist = Twist(Vector3(current_velocity_x, current_velocity_y, 0), Vector3(0, 0, current_velocity_theta))

    # publish the message
    odom_pub.publish(odom)

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

    global current_velocity_L, current_velocity_R
    newVelocity_L = pid_L(current_velocity_L)   # [m/s]
    newVelocity_R = pid_R(current_velocity_R)   # [m/s]

    newVelocity_L = newVelocity_L * (100 + wheelSpeedOffset)/100
    newVelocity_R = newVelocity_R * (100 - wheelSpeedOffset)/100

    #Convert new velocity [m/s] to RPM
    newRPM_L = (newVelocity_L / distancePerRevolution) * 60
    newRPM_R = (newVelocity_R / distancePerRevolution) * 60

    #Convert RPM to PWM-values
    newPWM_L = int((maxPWM/maxRPM) * newRPM_L)
    newPWM_R = int((maxPWM/maxRPM) * newRPM_R)

    newPWM_array.data = [newPWM_L, newPWM_R]

    pub_setVelocityPWM.publish(newPWM_array)

def driveStraight():

    velocity_offset = 0
    if delta_encoderTick_L > delta_encoderTick_R:
        velocity_offset = 0.1   # [m/s]
    elif delta_encoderTick_L < delta_encoderTick_R:
        velocity_offset = -0.1  # [m/s]

    global desiredVelocity_L, desiredVelocity_R
    desiredVelocity_L += velocity_offset    # [m/s]
    desiredVelocity_R += velocity_offset    # [m/s]

    updateVelocity()

def drive2Goal():
    a = 0

    r = rospy.Rate(20)

    while(a < 20000):

        #Goal position
        global desiredPosition_x, desiredPosition_y, x, y
        inc_x = desiredPosition_x - x
        inc_y = desiredPosition_y - y


        #print("desiredPosition_x: " + str(desiredPosition_x) + " desiredPosition_x: " + str(desiredPosition_y))
        #print("x: " + str(x) + ", y: " + str(y))
        #print("inc_x: " + str(inc_x) + " inc_x: " + str(inc_y))

        angleToGoal = math.atan2(inc_y, inc_x)    # [rad]
        #angleToGoal = angleToGoal *180/math.pi

        #theta_rad = theta * math.pi/180
        #if theta_rad >= math.pi or theta_rad <= -math.pi:
        #    theta_rad = 0

        global odom_quat
        #print("odom_quat: " + str(odom_quat))
        odom_euler = tf.transformations.euler_from_quaternion(odom_quat)
        theta = odom_euler[2]

        #global theta
        #print("angle_to_goal_from_current_position: " + str(angleToGoal))
        #print("Which way the robot faces (theta) [rad]: " + str(theta))
        #print("abs(angleToGoal - theta): " + str(abs(angleToGoal - theta)))

        
        global desiredVelocity, desiredVelocity_L, desiredVelocity_R

        distanceToGoal = math.sqrt(inc_x*inc_x + inc_y*inc_y)
        if distanceToGoal < 0.1:
            print("Goal has been reached")
            desiredVelocity_L = 0   # [m/s]
            desiredVelocity_R = 0   # [m/s]
            updateVelocity()
            return

        if abs(angleToGoal - theta) > 0.5:
            desiredVelocity_L = -0.5    # [m/s]
            desiredVelocity_R = 0.5     # [m/s]
        else:
            desiredVelocity_L = 1   # [m/s]
            desiredVelocity_R = 1   # [m/s]

        updateVelocity()

        a = a +1

        r.sleep()  
    
    print("2 I should stop now")
    desiredVelocity_L = 0   # [m/s]
    desiredVelocity_R = 0
    updateVelocity()


if __name__ == '__main__':
    rospy.init_node('node_motor', anonymous=True)

    setupSubscribers()

    global previous_time
    previous_time = rospy.Time.now()

    rospy.spin()

