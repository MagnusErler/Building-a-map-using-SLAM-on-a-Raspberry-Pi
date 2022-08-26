# Robocup2023

## Connect to RP3
```
ssh -X ubuntu@185.107.14.82
```
```-X``` starts up a proxy X11 server on the remote machine.

## Launch ROS-package

Data from Arduino
```
cd ~/Robocup2023/
roslaunch data_from_arduino talker.launch
```