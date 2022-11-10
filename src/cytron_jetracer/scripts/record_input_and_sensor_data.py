#!/usr/bin/env python3

import rospy
import pygame
import time
from std_msgs.msg import Float32, Float32MultiArray
import csv
import datetime
import os


class record_input_and_sensor_data:
	def __init__(self, car_number):

		self.car_number = car_number

		#initiate this node
		rospy.init_node('data_recording' + str(car_number), anonymous=True)
		
		#initialize variables
		self.safety_value = 0.0
		self.throttle = 0.0
		self.steering = 0.0
		self.current =  0.0
		self.voltage =  0.0
		self.IMU =  [0.0 , 0.0, 0.0]
		self.velocity =  0.0

		#subscribe to inputs and sensor in formation topics
		rospy.Subscriber('safety_value', Float32, self.callback_safety)
		rospy.Subscriber('throttle_' + str(car_number), Float32, self.callback_throttle)
		rospy.Subscriber('steering_' + str(car_number), Float32, self.callback_steering)
		rospy.Subscriber('current_' + str(car_number), Float32, self.callback_current)
		rospy.Subscriber('voltage_' + str(car_number), Float32, self.callback_voltage)
		rospy.Subscriber('IMU_' + str(car_number), Float32MultiArray, self.callback_IMU)
		rospy.Subscriber('velocity_' + str(car_number), Float32, self.callback_velocity)

		#also publish the recorded data
		self.pub_data_line = rospy.Publisher("data_to_store_" + str(car_number), Float32MultiArray, queue_size=1)
		


	
	
	
	def run_game_pad(self):
		rate = rospy.Rate(10) # 10hz
		date_time = datetime.datetime.now()
		date_time_str = date_time.strftime("%m_%d_%Y_%H_%M_%S")
		#date_time = date_time.strftime("%H_%M_%S")
		subfolder = '/Data_step/'
		current_dir = os.path.realpath(os.path.dirname(__file__))
		file_name = current_dir + subfolder + 'recording_'+ date_time_str + '.csv'
		print(file_name)
		start_clock_time = rospy.get_rostime()

		with open(file_name, 'w+') as file:
			writer = csv.writer(file)
			writer.writerow(['safety_value', 'throttle', 'steering', 'current', 'voltage', 'IMU[0]', 'IMU[1]', 'IMU[2]', 'velocity','elapsed time'])
			data_msg = Float32MultiArray()
			while not rospy.is_shutdown():
				#store data here
				stop_clock_time = rospy.get_rostime()
				elapsed_time = stop_clock_time.secs - start_clock_time.secs + (stop_clock_time.nsecs - start_clock_time.nsecs)/1000000000
				data_line = [self.safety_value, self.throttle, self.steering, self.current, self.voltage, self.IMU[0], self.IMU[1], self.IMU[2], self.velocity, elapsed_time]
				writer.writerow(data_line)
				#print(stop_clock_time)
				data_msg.data = data_line
				self.pub_data_line.publish(data_msg)
				rate.sleep()
				
		
		#file.close()

		

	#Safety callback function
	def callback_safety(self, safety_val_incoming):
		self.safety_value = safety_val_incoming.data

	#Throttle callback function
	def callback_throttle(self, throttle_data):
		self.throttle = throttle_data.data

	#Steering callback function
	def callback_steering(self, steering_data):
		self.steering = steering_data.data

	#Current callback function
	def callback_current(self, current_data):
		self.current =  current_data.data

	#Voltage callback function
	def callback_voltage(self, voltage_data):
		self.voltage =  voltage_data.data

	#IMU callback function
	def callback_IMU(self, IMU_data):
		self.IMU =  IMU_data.data

	#Velocity callback function
	def callback_velocity(self, velocity_data):
		self.velocity =  velocity_data.data


if __name__ == '__main__':
	try:
		car_number = 3
		recording = record_input_and_sensor_data(car_number)
		recording.run_game_pad()
	except rospy.ROSInterruptException:
		pass
