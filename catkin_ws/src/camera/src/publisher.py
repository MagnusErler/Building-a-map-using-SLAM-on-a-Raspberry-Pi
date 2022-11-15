#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_message():
  pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
  #pub = rospy.Publisher('/cam0/image_raw', Image, queue_size=10)

  rospy.init_node('video_pub_py', anonymous=True)

  # Go through the loop 2 times per second
  rate = rospy.Rate(2)  # 2 Hz

  cap = cv2.VideoCapture(0)
  #cap = cv2.VideoCapture('../video/ros.mp4')

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  while not rospy.is_shutdown():

      # Capture frame-by-frame
      # This method returns True/False as well as the video frame.
      ret, frame = cap.read()

      if ret == True:
        rospy.loginfo('publishing video frame')

        # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS image message
        pub.publish(br.cv2_to_imgmsg(frame))

      rate.sleep()

if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
