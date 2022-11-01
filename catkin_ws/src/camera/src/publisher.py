#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_message():

  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)

  # Go through the loop 2 times per second
  rate = rospy.Rate(2) # 10hz

  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  cap = cv2.VideoCapture(0)
  #cap = cv2.VideoCapture('../video/ros.mp4')

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # While ROS is still running.
  while not rospy.is_shutdown():

      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      ret, frame = cap.read()

      if ret == True:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')

        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(br.cv2_to_imgmsg(frame))

      # Sleep just enough to maintain the desired rate
      rate.sleep()

if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
