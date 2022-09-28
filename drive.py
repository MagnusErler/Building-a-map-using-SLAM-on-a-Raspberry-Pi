
#!/usr/bin/env python

# ROS
import rospy
from std_msgs.msg import Float32


pub_setSpeedPWM = rospy.Publisher('motor/CmdSetSpeed', Float32, queue_size=10)

def drive(speed):
    pub_setSpeedPWM.publish(speed)

def turn(speed):
    pub_setSpeedPWM.publish(speed)

if __name__ == '__main__':
    rospy.init_node('node_drive', anonymous=True)

    driveStraight(1)

    rospy.spin()
