


# Check COM port on windows machine (ex. COM4)
# Run command:		python .\showingLidarDataOnGraph_windows.py COM4




#!/usr/bin/env python3
import matplotlib.pyplot as plt
import sys
import serial
import time
from enum import Enum
import math

SERIAL_MODE = True

class State(Enum):
	START1 = 0
	START2 = 1
	HEADER = 2
	DATA = 3


def readbytes(file, count):
	data = f.read(count)
	if len(data) != count:
		print("End of file")
		print("len(data):")
		print(len(data))
		print("count: ")
		print(count)
		return False
	return data


if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("mising length arg")
		exit()

	name = str(sys.argv[1])
		
	try:
		if SERIAL_MODE:
			f = serial.Serial(name, 153600, timeout=0.1) 
			time.sleep(1)
#			flag = f.is_open
#			if flag:
#				print('success\n')
#				ser.close()
#			else:
#				print('Open Error\n')
#				ser.close() 
		else:
			f = open(name, "rb")
	except:
		print("could not connect to device")
		exit()
	counter = 0
	rmax = 10.0
	
	dist = plt.subplot(111, polar = True)
	
	data_lenght = 0
	start_angle = 0
	stop_angle = 0


	run = True
	step = (-math.pi*2) 
	try:
		state = State.START1
		while run:

			if state == State.START1:
				data = readbytes(f, 1)
				if data == False:
					break
				if data[0] == 0xAA:
					state = State.START2
				# else:
					# print("sync")
				continue
			elif state == State.START2:
				data = readbytes(f, 1)
				if data == False:
					break
				if data[0] == 0x55:
					state = State.HEADER
				else:
					state = State.START1
					# print("sync2")
				continue
			elif state == State.HEADER:
				data = readbytes(f, 8)
				if data == False:
					break
				pack_type = data[0]
				data_lenght = int(data[1])
				start_angle = int(data[3] << 8) + int(data[2])
				stop_angle = int(data[5] << 8) + int(data[4])
				#unknown = int(data[7] << 8) + int(data[6])

				diff = stop_angle - start_angle
				if stop_angle < start_angle:
					diff =  0xB400 - start_angle + stop_angle

				angle_per_sample = 0
				if diff > 1:
					angle_per_sample = diff / (data_lenght-1)
				
				# print("[{}]\ttype:{},\tlenght {},\tstart: {},\tstop: {}, \tdiff: {} \tdiff: {}".format(counter, pack_type, data_lenght, start_angle, stop_angle, diff, angle_per_sample), end="\n")
				
				counter += 1
				if pack_type != 0x2C:
					counter = 0

				state = State.DATA
				continue
				
			elif state == State.DATA:
				state = State.START1
				#read data
				data = readbytes(f, data_lenght * 3) 
				if data == False:
					break
		
				angle_list = []
				distance_list = []
				quality_list = []
		
				for i in range(0, data_lenght):
					
					data0 = int(data[i*3 + 0])
					data1 = int(data[i*3 + 1])
					data2 = int(data[i*3 + 2]) 
					distance = (data2  << 8) + data1

					angle = (start_angle + angle_per_sample * i)
					anglef = step * (angle / 0xB400) 
					angle_list.append(anglef)

					distance_list.append(distance / 1000) # div to convert mm to meter
					quality_list.append(data0) 
				
				if pack_type != 0x2C:
					plt.cla()
					continue # package contance no data skip plot

				dist.scatter(angle_list, distance_list, c = "purple", s = 3) # dot
				#dist.plot(angle_list, distance_list, c = "purple") # line 
				dist.scatter(angle_list, quality_list, c = "red", s = 1)

				dist.set_theta_offset(math.pi / 2)
				dist.set_rmax(rmax)
				dist.set_rmin(0.0)
				plt.pause(0.01)
				rmax = dist.get_rmax()
				
			else:
				print("error")
	
	except KeyboardInterrupt:
		run = False
		exit()
