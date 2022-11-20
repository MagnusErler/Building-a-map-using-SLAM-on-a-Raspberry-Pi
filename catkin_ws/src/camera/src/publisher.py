#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
cap.set(cv2.CAP_PROP_FPS,20)

rospy.init_node('node_image', anonymous=True)
pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

# Used to convert between ROS and OpenCV images
br = CvBridge()

rate = rospy.Rate(10)

while(cap.isOpened()):

    ret, frame = cap.read()

    # if video finished or no Video Input
    if not ret:
        break

    # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS image message
    pub.publish(br.cv2_to_imgmsg(frame, "rgb8"))

    rospy.loginfo('publishing video frame')

    rate.sleep()  

