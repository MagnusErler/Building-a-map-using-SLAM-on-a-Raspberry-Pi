<a name="readme-top"></a>

# Building a map using SLAM on a Raspberry Pi

## Setup personal machine to communicate wih ROS + RP4
<details>
<summary><b>Setup Ubuntu 20.04 Desktop on RP4</b></summary>
<br>
Because the latest ROS distribution (ROS Noetic Ninjemys) is not supported by Ubuntu 22.04 we will install Ubuntu 20.04.

For ROS we will install the desktop version of Ubuntu (Ubuntu 20.04 Desktop) as this will help us more with visualizing ROS.

There never was a specific desktop version of Ubuntu 20.04 for Raspberry Pi. Instead, we have to install the server version of Ubuntu 20.04, and when that is installed, install the desktop environment from the terminal.

Start with downloading Raspberry Pi Imager on your personal machine, insert your SD card and flash it with Ubuntu 20.04 Server. Insert the SD card into the RP4, connect display and HDMI cable, login (login: ubuntu, password: ubuntu), and change the password. Now install the desktop version (without all the bloat) with:

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
sudo systemctl start lightdm.service    #You are maybe forced to press ctrl+alt+F2 to come back to the terminal
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

PlatformIO is used to upload code to arduino from the terminal
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


```sudo chmod a+rw /dev/i2c-*``` is temporary a solution and is lost at next boot. To fix it permanently you need to do the following: https://lexruee.ch/setting-i2c-permissions-for-non-root-users.html

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

<details>
<summary>Calibrate camera</summary>
<br>
Install calibration package

Terminal 1
```
sudo apt-get install ros-noetic-camera-calibration
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install camera_calibration

cd RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/
python publisher.py 
```

Terminal 2
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.026 image:=/camera/image_raw camera:=/camera/image_raw --no-service-check
```

</details>

<br />

</details>

<details>
<summary><b>Setup ORB-SLAM3</b></summary>
<br>

OpenCV

```
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_GTK=ON -D WITH_OPENGL=ON ..
make
sudo make install
```

Pangolin
  
```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake -B build
cmake --build build
```
  
ORB_SLAM3

```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
chmod +x build_ros.sh
./build_ros.sh
```

<b>Run ORB-SLAM3-script</b>
  
```
cd ORB_SLAM3/
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt ./Examples/Monocular/TUM1.yaml 
```

<b>Run ROS ORB-SLAM3-script</b>

Terminal 1

```
cd RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/camera/src/
python publisher.py 
```

Terminal 2

```
cd ORB_SLAM3/
rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml
```

  
https://robot-vision-develop-story.tistory.com/10

</details>

<details>
<summary><b>SSH</b></summary>
<br>
SSH onto RP4

```
ssh -X ubuntu@185.107.14.82
```
```-X``` starts up a proxy X11 server on the remote machine.

When ssh-ing with GUI from a windows computer follow these steps:

- Launch XMing on Windows client
- Launch Putty
    * Fill in basic options as you know in session category
    * Connection -> SSH -> X11
        -> Enable X11 forwarding
        -> X display location = :0.0
        -> MIT-Magic-Cookie-1
        -> X authority file for local display = point to the Xming.exe executable

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
<br />

<details>
<summary>Get Data from Arduino</summary>
<br>

```
rostopic list
rostopic echo /battery/voltage
```
</details>

<details>
<summary>Send Data to Arduino</summary>
<br>

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
Arduino Mega 2560
```
pio run -e megaatmega2560 -t upload
```
</details>

<details>
<summary>Create a ROS package</summary>
<br>
Follow http://wiki.ros.org/ROS/Tutorials/CreatingPackage
</details>

<details>
<summary>Create script to launch at Boot</summary>
<br>
Follow https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/

```
sudo apt-get install ros-noetic-robot-upstart 
```

<b>Install script to launch at boot</b>
```
cd RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/catkin_ws/src/
rosrun robot_upstart install ros_setup/launch/ros_setup.launch --job ros_setup --symlink
sudo systemctl daemon-reload 
```

<b>Disabling</b>

```
sudo systemctl disable OLEDDisplay.py.service
```

<b>Re-enable</b>

```
sudo systemctl enable OLEDDisplay.py.service
```
</details>

<details>
<summary>Kill all ROS nodes</summary>

```
rosnode kill --all
```

</details>

<br />

### Communication (Subscriber and publisher)

#### <b>Publisher</b>
| Command  | Data types | Action |
| ------------- | ------------- | ------------- |
| ```/battery/voltage```  | Float32MultiArray  | Give the voltage of the battery for the motors and RP |
| ```/camera/image_raw```  | Image  | Image from Raspberry Pi camera |
| ```/IMU/temperature```  | Float32  | Give the temperature (from the MPU6050-chip) |
| ```/IMU/orientation```  | Int16MultiArray  | Give the orientation in degrees [pitch, roll, yaw] |
| ```/joystick```  | String  | Give pressed and released keys and values from the joystick |
| ```/motor/encoderTick```  | Int16MultiArray  | Give the encoder ticks for the two wheels [L, R] |
| ```/motor/odom```  | Odometry  | Give the odometry of the robot (position, orientation, and linear- and angular velocity) |

#### <b>Subscriber</b>
| Command  | Data types | Action | Example |
| ------------- | ------------- | ------------- | ------------- |
| ```/IMU/CmdCaliIMU```  | Empty  | Calibrate the IMU | ```rostopic pub /IMU/CmdCaliIMU std_msgs/Empty```
| ```/motor/CmdResetOdom```  | Empty  | Reset the odometry | ```rostopic pub /motor/CmdResetOdom std_msgs/Empty```
| ```/motor/CmdSetEvent```  | String  | Set an event, when to stop the motors | ```rostopic pub /motor/CmdSetEvent std_msgs/String dist=1.2``` (drive 1.2m).
| ```/motor/CmdSetTurnRadius```  | Float32  | Set the turning radius [m]. O.B.S. set first velocity. | ```rostopic pub /motor/CmdSetTurnRadius std_msgs/Float32 0.4``` (set the turning radius to 0.4 m). Setting the turning radius to 0 will have the robot to spin around its center 
| ```/motor/CmdSetVelocity```  | Float32  | Set the velocity of both motors [m/s] | ```rostopic pub /motor/CmdSetVelocity std_msgs/Float32 1.2``` (set the overall wheel velocity to 1.2 m/s). Use -- before negativ values (e.g. -- -4).
| ```/motor/CmdSetVelocityPWM```  | Int16MultiArray  | Set the velocity of both motors (0 = off and 255 = max speed). Negative values will drive the motor backwards |
| ```/OLED/CmdSetText```  | String  | Write 1 line of text to one of the 8 lines on the OLED display. Line 1-4 are reserved for IP-address, CPU Load, Memory, and Disk. The display updates every 1 sec with existing values | ```rostopic pub /OLED/CmdSetText std_msgs/String 5_Robot``` (writes <i>Robot</i> to line 5). Use " when writing multiple words.

## Other

### Shutdown RP
Don't just pull the plug. Use instead:

```
sudo shutdown -h now
```

### LiDAR
```
cd ~/RoboCup2023/Building-a-map-using-SLAM-on-a-Raspberry-Pi/
roslaunch mb_1r2t_ros view.launch port:=/dev/ttyUSB1
```
If ```ttyUSB1``` can't be found look for the USB-device with ```ls /dev/tty*```

This will open RViz and show the LiDAR data

### Control power to RP4 USB-ports
https://github.com/mvp/uhubctl#raspberry-pi-b2b3b

### Microcontroller

Minimum requirements:
- Ports
    - Digital input (interrupt): 4
    - Digital output (PWM): 6
    - Analog input: 1
    - Analog output: 0
    - I2C: 1
- Storage
    - RAM: 3 kB
    - Flash: 23 kB

<p align="right">(<a href="#readme-top">back to top</a>)</p>
