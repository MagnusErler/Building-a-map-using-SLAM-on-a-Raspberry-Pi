#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

pub = rospy.Publisher('motor/CmdSetSpeed', Int16MultiArray, queue_size=10)

speed_float = Int16MultiArray()
speed_float.data = []

def callback(data):

    try:
        [key, value] = data.data.split(": ")
    except:
        key = data.data.split(": ")
        value = 0

    value = int(float(value)*255)

    if (key == "ry"):
        speed_float.data = [value,value]
    elif (key == "rx"):
        if (value < 0):
            speed_float.data = [-value,value]
        else:
            speed_float.data = [-value,value]

    pub.publish(speed_float)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joystick", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass