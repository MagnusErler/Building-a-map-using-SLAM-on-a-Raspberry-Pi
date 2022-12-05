import cv2
import time

input_video_path = '/home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/camera_scripts/recordedVideo.avi'

cap = cv2.VideoCapture(input_video_path)
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
cap.set(cv2.CAP_PROP_FPS,20)

cv2.namedWindow('frame', flags=cv2.WINDOW_GUI_NORMAL)

# For fps
global prev_frame_time, new_frame_time
prev_frame_time = 0
new_frame_time = 0

def getFPS():
    return cap.get(CAP_PROP_FPS)
    # time when we finish processing for this frame
    global prev_frame_time, new_frame_time
    new_frame_time = time.time()
 
    # Calculating the fps
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
 
    return "FPS: " + str(int(fps))

while(cap.isOpened()):
    cv2.waitKey(1)

    ret, frame = cap.read()

    # if video finished or no Video Input
    if not ret:
        break

    # putting the FPS count on the frame
    #cv2.putText(frame, str(cap.get(cv2.CAP_PROP_FPS)), (1, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 1, cv2.LINE_AA)

    cv2.imshow('frame', frame)

cap.release()
cv2.destroyAllWindows()
print("Closing video output")
