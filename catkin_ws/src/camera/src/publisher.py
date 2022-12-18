#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

if __name__ == '__main__':

    # ROS
    rospy.init_node('node_image', anonymous=True)
    pub = rospy.Publisher('/cam0/image_raw', Image, queue_size=1)

    rate = rospy.Rate(50)

    # CAMERA
    input_video_path = '/home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/camera_scripts/recordedVideo.avi'
    
    width = int(3280/8)
    height = int(2464/8)

    #cap = cv2.VideoCapture(input_video_path)
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
    cap.set(cv2.CAP_PROP_FPS,20)

    br = CvBridge() # Used to convert between ROS and OpenCV images

    if (cap.isOpened()):
        rospy.loginfo('Publishing video feed...')
    else:
        rospy.loginfo("Can't open camera video feed")

    while not rospy.is_shutdown():

        if (cap.isOpened()):     # Checking if cap object has started capturing the frames

            ret, frame = cap.read()

            if not ret: # Checking if the actual frame isn’t corrupt
                break

            # Converting OpenCV image to ROS image and publishing as an 8bit BGR image
            pub.publish(br.cv2_to_imgmsg(frame, "bgr8"))
        else:
            rospy.loginfo("Can't open camera video feed")

        rate.sleep()  
