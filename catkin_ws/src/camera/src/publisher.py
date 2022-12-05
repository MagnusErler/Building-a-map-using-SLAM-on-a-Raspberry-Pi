#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

if __name__ == '__main__':

    # ROS
    rospy.init_node('node_image', anonymous=True)
    #pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
    pub1 = rospy.Publisher('/cam0/image_raw', Image, queue_size=1)
    #pub2 = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)

    rate = rospy.Rate(50)

    # CAMERA
    input_video_path = '/home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/camera_scripts/recordedVideo.avi'

    cap = cv2.VideoCapture(input_video_path)
    #cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 300)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
    cap.set(cv2.CAP_PROP_FPS,20)

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    if (cap.isOpened()):
        rospy.loginfo('Publishing video feed...')
    else:
        rospy.loginfo("Can't open camera video feed")
    
    while not rospy.is_shutdown():

        if (cap.isOpened()):

            ret, frame = cap.read()

            # if video finished or no Video Input
            if not ret:
                break

            if frame.shape[0] != 300 and frame.shape[1] != 300 and frame.shape[2] != 3:
                rospy.loginfo("Not correct size")
                continue

            # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS image message
            #pub.publish(br.cv2_to_imgmsg(frame, "rgb8"))
            pub1.publish(br.cv2_to_imgmsg(frame, "rgb8"))
            #pub2.publish(br.cv2_to_imgmsg(frame, "rgb8"))

        #     rospy.loginfo('Publishing video frame')
        # else:
        #     rospy.loginfo("Can't open camera video feed")

            
        rate.sleep()  

