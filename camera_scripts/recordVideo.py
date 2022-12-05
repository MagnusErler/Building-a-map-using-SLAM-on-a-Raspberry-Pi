
   
import cv2
  
cap = cv2.VideoCapture(0)
   
if (cap.isOpened() == False): 
    print("Error reading video file")
  
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
   
size = (frame_width, frame_height)
   
result = cv2.VideoWriter('recordedVideo.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

print("Recording video...")
    
while(True):
    ret, frame = cap.read()

    if not ret:
        break

    result.write(frame)

    # Press S on keyboard 
    # to stop the process
    if cv2.waitKey(1) & 0xFF == ord('s'):
        break
  
cap.release()
result.release()

cv2.destroyAllWindows()
   
print("The video was successfully saved")