#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Bool
from serial_read import Readline
import os
import time
import serial
import numpy as np


class PubSensors:
	
	def __init__(self, car_number):
		#Setup node and topics subscription
		print("Sensor publisher setup ros topics and node")

		#setup this node
		rospy.init_node('publish_sensors_'+str(car_number), anonymous=True)
		#setup publisher handles
		pub_sens = rospy.Publisher("sensors_" + str(car_number), Float32MultiArray, queue_size=1)

		try:
			print("Setting up USB connection")
			arduino = serial.Serial(port="/dev/arduino", baudrate=115200, timeout=1)
			print("Setting up USB connection done")
			Rline = Readline(arduino)
		except:
			print('Please check the USB port connections')
		
		print('waiting for usb connection to be ready')
		r = rospy.Rate(0.2)
		r.sleep()


		print("starting sensor data publishing")
		# Run the while loop for the controller
		rate = rospy.Rate(20)	#frequency !! note that the serial gives new measurements at 10 Hz so keep it like this, faster rates seem to bee too much for the battery sensor and the values stop being updated
		start_clock_time = rospy.get_rostime()
		while not rospy.is_shutdown():

			
			#Check for new serial data
			data = Rline.readline()
			if data:

				#find indexes to extract data
				acc_x_index = data.find('Acc_x')
				acc_y_index = data.find('Acc_y')
				gyr_z_index = data.find('Gyr_z')
				vel_index = data.find('Vel')

				# extract data
				current = float(data[3:7])
				voltage = float(data[10:14])
				acc_x = float(data[acc_x_index+5 : acc_y_index])
				acc_y = float(data[acc_y_index+5 : gyr_z_index])
				omega_rad = float(data[gyr_z_index+5 : vel_index]) 			
				vel = float(data[vel_index+3:])


				stop_clock_time = rospy.get_rostime()
				elapsed_time = stop_clock_time.secs - start_clock_time.secs + (stop_clock_time.nsecs - start_clock_time.nsecs)/1000000000
				time_now = stop_clock_time.secs + (stop_clock_time.nsecs)/1000000000

				#define messages to send
				sensor_msg = Float32MultiArray()

				# safety_value, current,voltage,IMU[0](acc x),IMU[1] (acc y),IMU[2] (omega rads),velocity
				sensor_msg.data = [elapsed_time, current, voltage, acc_x,acc_y, omega_rad, vel]

				#publish messages
				pub_sens.publish(sensor_msg)
			

			# Sleep for the time set in the rate
			rate.sleep()
		
		

			
		

if __name__ == '__main__':
	print("Starting pid-controller for velocity")
	try:	
		car_number = os.environ["car_number"]
		vel_publisher = PubSensors(car_number)
	except rospy.ROSInterruptException:
		print('failed to lauch velocity publisher')
