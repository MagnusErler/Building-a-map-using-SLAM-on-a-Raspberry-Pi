
#!/usr/bin/env python

# ROS
import rospy
from std_msgs.msg import Float32


pub_setVelocityPWM = rospy.Publisher('motor/CmdSetVelocity', Float32, queue_size=10)

def drive(velocity):
    pub_setVelocityPWM.publish(velocity)

def command(command):
    [driveData, eventData] = command.split(" : ")

    [velocity, velocityValue] = driveData.split("=")

if __name__ == '__main__':
    rospy.init_node('node_drive', anonymous=True)

    command("vel=1.2 : dist=0.4")

    driveStraight(1)

    #snprintf(lines[line++], MAX_LEN, "vel=0.40: dist=0.4");

    rospy.spin()
