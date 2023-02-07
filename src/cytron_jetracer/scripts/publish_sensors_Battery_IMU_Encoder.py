#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Bool
from serial_read import Readline
import os
import time
import serial


class PubSensors:
	
	def __init__(self, car_number):
		#Setup node and topics subscription
		print("setup ros topics and node")

		#setup this node
		rospy.init_node('publish_velocity_'+str(car_number), anonymous=True)
		#setup publisher handles
		pub_cur = rospy.Publisher("current_" + str(car_number), Float32, queue_size=1)
		pub_vol = rospy.Publisher("voltage_" + str(car_number), Float32, queue_size=1)
		pub_acc = rospy.Publisher("IMU_" + str(car_number), Float32MultiArray, queue_size=1)
		pub_vel = rospy.Publisher("velocity_" + str(car_number), Float32, queue_size=1)
		
		#Start serial connection
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

		#self.Vel = 0
		
		
		print("starting sensor data publishing")
		# Run the while loop for the controller
		rate = rospy.Rate(10)	#frequency !! note that the serial gives new measurements at 10 Hz so keep it like this, faster rates seem to bee too much for the battery sensor and the values stop being updated
		while not rospy.is_shutdown():
			
			#Check for new serial data
			data = Rline.readline()
			if data:
			#	data = float(data)
				#if data != data_prev:
			#self.Vel = data/1000
				#	data_prev = data
				#Publish the new measured velocity
			#pub_vel.publish(self.Vel)

				#find indexes to extract data
				acc_x_index = data.find('Acc_x')
				acc_y_index = data.find('Acc_y')
				gyr_z_index = data.find('Gyr_z')
				# etract data
				vel_index = data.find('Vel')
				current = float(data[3:7])
				voltage = float(data[10:14])
				acc = [float(data[acc_x_index+5 : acc_y_index]), float(data[acc_y_index+5 : gyr_z_index]), float(data[gyr_z_index+5 : vel_index]) ]
				vel = float(data[vel_index+3:])


				#define messages to send
				acc_msg = Float32MultiArray()
				acc_msg.data = acc

				#publish messages
				pub_cur.publish(current)
				pub_vol.publish(voltage)
				pub_acc.publish(acc_msg)
				pub_vel.publish(vel)
			

			# Sleep for the time set in the rate
			rate.sleep()
		
		

			
		

if __name__ == '__main__':
	print("Starting pid-controller for velocity")
	try:	
		car_number = os.environ["car_number"]
		vel_publisher = PubSensors(car_number)
	except rospy.ROSInterruptException:
		print('failed to lauch velocity publisher')
