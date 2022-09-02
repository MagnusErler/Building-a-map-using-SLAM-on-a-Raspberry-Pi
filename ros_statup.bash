#!/bin/bash
#roslaunch data_from_arduino talker.launch
roslaunch joystick talker_joystick.launch
rosrun rosserial_python serial_node.py /dev/ttyACM0
