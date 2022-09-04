<a name="readme-top"></a>

# Robocup2023

## Connect to RP4

### SSH
```
ssh -X ubuntu@185.107.14.82
```
```-X``` starts up a proxy X11 server on the remote machine.

### SSH folder from RP onto personal computer
- Create folder (e.g. RP4_files)
```
sshfs ubuntu@185.107.14.82:/ /home/magnus/RP4_files/
```

### SSH in Visual Studio Code
- Open VSC
- Click on green box ('Open a Remote Window') in lower left corne
- Click 'Connect to Host...'
- ssh ubuntu@185.107.14.82

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

## Uploade Arduino-code from terminal
Arduino Micro Pro
```
pio run -e micro -t upload
```

Arduino Mega 2560
```
pio run -e megaatmega2560 -t upload
```

## Communicate with Arduino (Rosrun)
Terminal 1:
```
roscore
```

Terminal 2:
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
If ```ttyUSB0``` can't be found look fo the USB-device with ```ls /dev/tty*```

### Get Data from Arduino (Rosrun)
Terminal 3:
```
rostopic list
rostopic echo /voltage
```

### Send Data to Arduino (Rosrun)
Terminal 3:
```
rostopic list
rostopic pub /CmdSetMotor std_msgs/UInt16 255
```
(Controlling speed: 0 = off and 255 = max speed)

## Launch ROS-package

<b>Joystick</b>
```
roslaunch joystick talker_joystick.launch
```

## LiDAR
```
cd ~/Robocup2023/
roslaunch mb_1r2t_ros view.launch port:=/dev/ttyUSB1
```
If ```ttyUSB1``` can't be found look fo the USB-device with ```ls /dev/tty*```
This will open RViz and show the LiDAR data

## Control power to RP4 USB-ports
https://github.com/mvp/uhubctl#raspberry-pi-b2b3b

<p align="right">(<a href="#readme-top">back to top</a>)</p>
