#!/usr/bin/env python

# ROS
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String


pub_setVelocity = rospy.Publisher('/motor/CmdSetVelocity', Float32, queue_size=10)
pub_setEvent = rospy.Publisher('/motor/CmdSetEvent', String, queue_size=10)

def command(command):
    [driveData, eventData] = command.split(" : ")

    print(driveData)
    print(eventData)

    [velocity, velocityValue] = driveData.split("=")

    pub_setEvent.publish(eventData)
    pub_setVelocity.publish(float(velocityValue))

if __name__ == '__main__':
    rospy.init_node('node_drive', anonymous=True)

    command("vel=1.2 : dist=1")

    rospy.spin()


