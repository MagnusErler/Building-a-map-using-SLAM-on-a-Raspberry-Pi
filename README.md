# Robocup2023

## Connect to RP3
```
ssh -X ubuntu@185.107.14.82
```
```-X``` starts up a proxy X11 server on the remote machine.

## Launch ROS-package

```
cd ~/Robocup2023/
```

***Remember to source .bash-file ```source /opt/ros/noetic/setup.bash```

<b>Data from Arduino</b>
```
roslaunch data_from_arduino talker.launch
```

<b>Joystick</b>
```
roslaunch joystick talker_joystick.launch
```
