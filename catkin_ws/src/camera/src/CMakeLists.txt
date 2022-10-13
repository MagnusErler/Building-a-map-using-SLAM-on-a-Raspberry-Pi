cmake_minimum_required(VERSION 2.8)
project( camera )
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
  )
find_package(OpenCV 4.2.0 REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
add_executable(camera camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})