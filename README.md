<a name="readme-top"></a>

# Robocup2023

## Setup personal machine to communicate wih ROS + RP4
<details>
<summary>Setup Ubuntu 20.04 on RP4</summary>
<br>
Because the latest ROS distro (Noetic) is not supported by Ubuntu 22.04 we will install Ubuntu 20.04.

For ROS we will install the desktop version of Ubuntu (Ubuntu 20.04 Desktop) as this will help us more with visualizing ROS.

There never was a specific desktop version of Ubuntu 20.04 for Raspberry Pi. Instead, we have to install the server version of Ubuntu 20.04, and when that is installed, install the desktop environment from terminal

Start with downloading Raspberry Pi Imager, insert you SD card and flash it with Ubuntu 20.04 Server. Insert the SD card into the RP4, find the IP-address, ssh onto it, and change the password. Now install the desktop version (without all the bloat) with:

```
sudo apt-get install --no-install-recommends ubuntu-desktop
```
Install and setup LightDM as a Desktop Environment/GUI:
```
sudo apt install lightdm
sudo systemctl start lightdm.service    #You are maybe forced to press ctrl+alt+F2 to come back to the terminal)
sudo service ligthdm start
sudo reboot
```
Maybe below need to be added to the config.txt
```
hdmi_drive=2
hdmi_safe=1
dtoverlay=vc4-fkms-v3d
```
</details>

<details>
<summary>Setup ROS</summary>
<br>

For RP4 we will install ROS Noetic

http://wiki.ros.org/noetic/Installation/Ubuntu

Install further packages:
```
sudo apt-get install python3-roslaunch
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```

</details>

<details>
<summary>Setup PlatoformIO</summary>
<br>

PlatformIO is used to uploade code to arduino from the terminal
```
sudo apt-get install python3 python3-pip
sudo python3 -m pip install -U platformio
```

</details>

<details>
<summary>Setup OLED display</summary>
<br>

Install used packages
```
pip3 install Adafruit_GPIO
pip3 install adafruit-ssd1306
sudo apt-get install python3-dev python3-rpi.gpio

wget https://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20200601_all.deb -P /tmp
sudo apt-get install libnewt0.52 whiptail parted triggerhappy lua5.1 alsa-utils -y
sudo apt-get install -fy
sudo dpkg -i /tmp/raspi-config_20200601_all.deb
```
Follow these instructions to enable I2C interface: https://www.instructables.com/Raspberry-Pi-Monitoring-System-Via-OLED-Display-Mo/


</details>

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
source /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/devel/setup.bash
```

<b>Source .bash-file automatically</b>

1. ```nano ~/.bashrc```

2. Go to the bottom (last line should be something like ```source /opt/ros/noetic/setup.bash```)

3. Go under that line and write ```source /home/ubuntu/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/devel/setup.bash```

4. Save and exit

Now with every new shell you open, it will source automatically
</details>

## Start using ROS
Terminal 1:
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
Terminal 2:

```
rostopic list
rostopic echo /voltage
```
</details>

<details>
<summary>Send Data to Arduino (Rosrun)</summary>
<br>
Terminal 2:

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

### Communication (Subscriber and publisher)

#### <b>Publisher</b>
| Command  | Data types | Action |
| ------------- | ------------- | ------------- |
| ```/battery/voltage```  | Float32  | Gives the voltage of the battery |
| ```/IMU/temperatur```  | Float32  | Gives the temperature (from the MPU6050-chip) |
| ```/IMU/orientation```  | String  | Gives the orientation |

#### <b>Subscriber</b>
| Command  | Data types | Action | Example |
| ------------- | ------------- | ------------- | ------------- |
| ```/CmdSetPubFreq```  | UInt16  | Sets publishing rate |
| ```/IMU/CmdCaliIMU```  | Bool  | Calibrates the IMU (calibrate = true) |
| ```/motor/CmdSetMotor```  | Int16MultiArray  | Sets the speed of both motors (0 = off and 255 = max speed) |
| ```/OLED/sendText```  | String  | Write 1 line of text to 1 of the 8 lines on the OLED display. Line 1-4 are reserved for IP-address, CPU Load, Memory,and Disk. The display updates every 1 sec with existing values | ```rostopic pub /OLED/sendText std_msgs/String 5_Robot``` (writes "Robot" to line 5). Use " when writing multiple words.

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
