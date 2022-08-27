# Robocup2023

## Connect to RP3
```
ssh -X ubuntu@185.107.14.82
```
```-X``` starts up a proxy X11 server on the remote machine.

## Source .bash-file

Every terminal should source it automatically, if not you may do it manually

```
source /home/ubuntu/Robocup2023/catkin_ws/devel/setup.bash
```

<b>Source .bash-file automatically</b>

1. ```nano ~/.bashrc```

2. Go to the bottom (last line should be something like ```source /opt/ros/noetic/setup.bash```)

3. Go under that line and write ```source /home/ubuntu/Robocup2023/catkin_ws/devel/setup.bash```

4. Save and exit

Now with every new shell you open, it will source automatically

## Launch ROS-package

```
cd ~/Robocup2023/
```

<b>Data from Arduino</b>
```
roslaunch data_from_arduino talker.launch
```

<b>Joystick</b>
```
roslaunch joystick talker_joystick.launch
```

## Control power to RP3 USB-ports

https://github.com/mvp/uhubctl#raspberry-pi-b2b3b
