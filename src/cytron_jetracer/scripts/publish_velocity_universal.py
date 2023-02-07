#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from serial_read import Readline
import time
import serial
import os


class PubVelocity:
	
	def __init__(self):
		#Setup node and topics subscription
		print("setup ros topics and node")
		#Car number
		car_number = os.environ["car_number"]

		rospy.init_node('publish_velocity_'+str(car_number), anonymous=True)
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

		self.Vel = 0
		
		#data_prev = 0			#Check for new data from usb
		rate = rospy.Rate(10)	#frequency of controller
		
		print("starting velocity publishing")
		# Run the while loop for the controller
		while not rospy.is_shutdown():
			
			#Check for new serial data
			data = Rline.readline()
			if data:
				data = float(data)
				#if data != data_prev:
				self.Vel = data/1000
				#	data_prev = data
				#Publish the new measured velocity
				pub_vel.publish(self.Vel)
			


			# Sleep for the time set in the rate
			rate.sleep()
		
		

			
		

if __name__ == '__main__':
	print("Starting pid-controller for velocity")
	try:
		vel_publisher = PubVelocity()
	except rospy.ROSInterruptException:
		print('failed to lauch velocity publisher')
