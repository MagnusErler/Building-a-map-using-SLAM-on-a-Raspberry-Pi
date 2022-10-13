
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>

//#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Hello from ROS node " << ros::this_node::getName());

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  VideoCapture cap(0);

  cap.set(CAP_PROP_FRAME_WIDTH, 320);
  cap.set(CAP_PROP_FRAME_HEIGHT, 240);

  // Check if camera opened successfully
  if(!cap.isOpened()) {
   	cout << "Error opening video stream" << endl;
        return -1;
  }

  Mat frame;
  
  while(true) {
   
    // Capture frame-by-frame
    cap >> frame;
 
    // If the frame is empty, break immediately
    if (frame.empty()) break;

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg
   
    // Display the resulting frame    
    //imshow("Frame", frame);
 
    // Press ESC on keyboard to exit
    char c = (char)waitKey(25);
    if(c == 27) break;
  }

  // Closes all the frames
  destroyAllWindows();

  return 0;
}