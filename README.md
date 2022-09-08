<a name="readme-top"></a>

# Robocup2023

## Setup personal machine to communicate wih ROS + RP4
<details>
<summary>SSH</summary>
<br>
SSH onto RP4

```
ssh -X ubuntu@185.107.14.82
```
```-X``` starts up a proxy X11 server on the remote machine.
</details>

<details>
<summary>SSH folder from RP onto personal computer</summary>
<br>
Create folder (e.g. RP4_files)

```
sshfs ubuntu@185.107.14.82:/ /home/magnus/RP4_files/
```
</details>

<details>
<summary>SSH in Visual Studio Code</summary>
<br>
SSH onto RP4 via VSC

- Open VSC

- Click on green box ('Open a Remote Window') in lower left corner (If this can't be found, be sure that <b>Remote - SSH</b> is installed from Extensions)

- Click 'Connect to Host...'
  
- ubuntu@185.107.14.82
</details>

<details>
<summary>Source .bash-file</summary>
<br>
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
</details>

<br />

## Start using ROS
```
roslaunch ros_setup ros_setup.launch
```
This runs alle nodes for the <b>joystick</b>, <b>Arduino</b> and <b>OLED display</b>

OBS! ```roscore``` will be called automatically - no need to do that manually

If ```ttyUSB0``` can't be found look for the USB-device with ```ls /dev/tty*```

<br />

<details>
<summary>Get Data from Arduino (Rosrun)</summary>
<br>
Terminal 3:

```
rostopic list
rostopic echo /voltage
```
</details>

<details>
<summary>Send Data to Arduino (Rosrun)</summary>
<br>
Terminal 3:

```
rostopic list
rostopic pub /motor/CmdSetMotor std_msgs/UInt16 255
```
(Controlling speed: 0 = off and 255 = max speed)
</details>

<details>
<summary>Upload Arduino-code from terminal</summary>
<br>
Using platformio to send code to Arduino

```
cd Robocup2023/arduino/
```
Arduino Micro Pro
```
pio run -e micro -t upload
```

Arduino Mega 2560
```
pio run -e megaatmega2560 -t upload
```
</details>

<br />

### Communication (Subscriber and publisher)

#### <b>Publisher</b>
| Command  | Data types | Action |
| ------------- | ------------- | ------------- |
| ```/battery/voltage```  | Float32  | gives the voltage of the battery  |
| ```/IMU/temperatur```  | Float32  | gives the temperature (from the MPU6050-chip)  |
| ```/IMU/orientation```  | String  | gives the orientation  |

#### <b>Subscriber</b>
| Command  | Data types | Action |
| ------------- | ------------- | ------------- |
| ```/motor/CmdSetMotor```  | Int16MultiArray  | sets the speed of both motors (0 = off and 255 = max speed)  |
| ```/IMU/CmdCaliIMU```  | Bool  | calibrates the IMU (calibrate = true)  |
| ```/CmdSetPubFreq```  | UInt16  | sets publishing rate  |

## LiDAR
```
cd ~/Robocup2023/
roslaunch mb_1r2t_ros view.launch port:=/dev/ttyUSB1
```
If ```ttyUSB1``` can't be found look for the USB-device with ```ls /dev/tty*```

This will open RViz and show the LiDAR data

## Control power to RP4 USB-ports
https://github.com/mvp/uhubctl#raspberry-pi-b2b3b

<p align="right">(<a href="#readme-top">back to top</a>)</p>
