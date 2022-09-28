<a name="readme-top"></a>

# Robocup2023

## Setup personal machine to communicate wih ROS + RP4
<details>
<summary><b>Setup Ubuntu 20.04 Desktop on RP4</b></summary>
<br>
Because the latest ROS distribution (ROS Noetic Ninjemys) is not supported by Ubuntu 22.04 we will install Ubuntu 20.04.

For ROS we will install the desktop version of Ubuntu (Ubuntu 20.04 Desktop) as this will help us more with visualizing ROS.

There never was a specific desktop version of Ubuntu 20.04 for Raspberry Pi. Instead, we have to install the server version of Ubuntu 20.04, and when that is installed, install the desktop environment from the terminal.

Start with downloading Raspberry Pi Imager on your personal machine, insert your SD card and flash it with Ubuntu 20.04 Server. Insert the SD card into the RP4, connect display and HDMI cable, login (long: ubuntu, password: ubuntu), and change the password. Now install the desktop version (without all the bloat) with:

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install --no-install-recommends ubuntu-desktop
```
Install and setup LightDM as a Desktop Environment/GUI:
```
sudo apt-get install lightdm
```
Choose lightdm in the popup window
```
sudo systemctl start lightdm.service    #You are maybe forced to press ctrl+alt+F2 to come back to the terminal)
sudo service ligthdm start
sudo reboot
```
Add below to config.txt (This need to be done in a separate file ```sudo nano /boot/firmware/usercfg.txt```)
```
hdmi_drive=2
hdmi_safe=1
dtoverlay=vc4-fkms-v3d
```
Be aware that RP4 has 2 HDMI-outputs and some of the programs might open in HDMI1 (You should be connected to HDMI0).
</details>

<details>
<summary><b>Setup ROS</b></summary>
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
<summary><b>Setup PlatformIO</b></summary>
<br>

PlatformIO is used to uploade code to arduino from the terminal
```
sudo apt-get install python3 python3-pip
sudo python3 -m pip install -U platformio
```

</details>

<details>
<summary><b>Setup OLED display</b></summary>
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


sudo chmod a+rw /dev/i2c-*
This is temporary and is lost at next boot so, to fix it permanently you need to do the following: https://lexruee.ch/setting-i2c-permissions-for-non-root-users.html

</details>

<details>
<summary><b>Setup RP camera</b></summary>
<br>
Enable camera
  
```
sudo apt-get update
sudo apt-get upgrade
```
Edit /boot/firmware/config.txt, append the following config at the end of the file
```
start_x=1
gpu_mem=128
```
And comment out ```dtparam=i2c_arm=on```

Follow these instructions to enable the RP camera v2.1: https://zengliyang.wordpress.com/2021/01/04/raspberry-pi-4b-ubuntu-20-04-camera/
```
curl -L --output /usr/bin/rpi-update https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update && chmod +x /usr/bin/rpi-update
sudo rpi-update
sudo apt install cmake
git clone https://github.com/raspberrypi/userland.git
cd userland
./buildme # or "./buildme --aarch64" for 64-bit OS
touch ~/.bash_aliases
echo -e 'PATH=$PATH:/opt/vc/bin\nexport PATH' >> ~/.bash_aliases
echo -e 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vc/lib\nexport LD_LIBRARY_PATH' >> ~/.bash_aliases
source ~/.bashrc
sudo ldconfig
```
Give non-root users access to the camera device:
```
echo 'SUBSYSTEM==\"vchiq\",GROUP=\"video\",MODE=\"0660\"' > /etc/udev/rules.d/10-vchiq-permissions.rules
sudo usermod -a -G video $USER
sudo reboot
```
Test with ```raspistill -o test.jpg```
</details>

<details>
<summary><b>SSH</b></summary>
<br>
SSH onto RP4

```
ssh -X ubuntu@185.107.14.82
```
```-X``` starts up a proxy X11 server on the remote machine.

When ssh-ing with GUI from a windows computer follow these steps (Top answer): https://stackoverflow.com/questions/34932495/forward-x11-failed-network-error-connection-refused

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
</details>

<details>
<summary><b>Source .bash-file</b></summary>
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
cd RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/arduino/
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

<details>
<summary>Calibrate camera</summary>
<br>
Install calibration package

```
sudo apt-get install ros-noetic-camera-calibration
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install camera_calibration
```
</details>

<details>
<summary>Create a ROS package</summary>
<br>
Follow http://wiki.ros.org/ROS/Tutorials/CreatingPackage
</details>

<details>
<summary>Make a ROS Launch Start on Boot</summary>
<br>
Follow https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/
</details>

<br />

### Communication (Subscriber and publisher)

#### <b>Publisher</b>
| Command  | Data types | Action |
| ------------- | ------------- | ------------- |
| ```/battery/voltage```  | Float32  | Give the voltage of the battery |
| ```/IMU/temperatur```  | Float32  | Give the temperature (from the MPU6050-chip) |
| ```/IMU/orientation```  | String  | Give the orientation |
| ```/joystick```  | String  | Give pressed and released keys and values from the joystick |
| ```/motor/encoderTick```  | Int16MultiArray  | Give the encoder ticks for the two wheels [L, R] |
| ```/odom```  | Odometry  | Give the odometry of the robot (position, orientation, and linear- and angular velocity) |

#### <b>Subscriber</b>
| Command  | Data types | Action | Example |
| ------------- | ------------- | ------------- | ------------- |
| ```/CmdSetPubFreq```  | UInt16  | Set publishing rate |
| ```/IMU/CmdCaliIMU```  | Empty  | Calibrate the IMU | ```rostopic pub /IMU/CmdCaliIMU std_msgs/Empty```
| ```/motor/CmdResetOdom```  | Empty  | Reset the odometry | ```rostopic pub /motor/CResetOdom std_msgs/Empty```
| ```/motor/CmdSetEvent```  | String  | Set an event, when to stop the motors | ```rostopic pub /motor/CmdSetEvent std_msgs/String dist=1.2``` (drive 1.2m).
| ```/motor/CmdSetVelocity```  | Float32  | Set the velocity of both motors [m/s] | ```rostopic pub /motor/CmdSetVelocity std_msgs/Float32 1.2``` (set the overall wheel velocity to 1.2 m/s). Use -- before negativ values (e.g. -- -4).
| ```/motor/CmdSetVelocityPWM```  | Int16MultiArray  | Set the velocity of both motors (0 = off and 255 = max speed). Negative values will drive the motor backwards |
| ```/OLED/CmdsetText```  | String  | Write 1 line of text to one of the 8 lines on the OLED display. Line 1-4 are reserved for IP-address, CPU Load, Memory, and Disk. The display updates every 1 sec with existing values | ```rostopic pub /OLED/CmdsetText std_msgs/String 5_Robot``` (writes <i>Robot</i> to line 5). Use " when writing multiple words.

## LiDAR
```
cd ~/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/
roslaunch mb_1r2t_ros view.launch port:=/dev/ttyUSB1
```
If ```ttyUSB1``` can't be found look for the USB-device with ```ls /dev/tty*```

This will open RViz and show the LiDAR data

## Control power to RP4 USB-ports
https://github.com/mvp/uhubctl#raspberry-pi-b2b3b

<p align="right">(<a href="#readme-top">back to top</a>)</p>
