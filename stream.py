import cv2
import time

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# For fps
global prev_frame_time, new_frame_time
prev_frame_time = 0
new_frame_time = 0

def getFPS():
    # time when we finish processing for this frame
    global prev_frame_time, new_frame_time
    new_frame_time = time.time()
 
    # Calculating the fps
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
 
    return "FPS: " + str(int(fps))

while(cap.isOpened()):
    ret, frame = cap.read()

    # if video finished or no Video Input
    if not ret:
        break
 
    # resizing the frame size according to our need
    frame = cv2.resize(frame, (500, 300))
 
    # font which we will be using to display FPS
    font = cv2.FONT_HERSHEY_SIMPLEX

    # putting the FPS count on the frame
    cv2.putText(frame, getFPS(), (1, 15), font, 0.5, (100, 255, 0), 1, cv2.LINE_AA)

    cv2.namedWindow('frame', flags=cv2.WINDOW_GUI_NORMAL)
    cv2.imshow('frame', frame)

    # press 'Q' if you want to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Closing video output")