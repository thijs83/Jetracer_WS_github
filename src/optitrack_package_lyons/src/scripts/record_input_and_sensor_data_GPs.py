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
		self.Jet_data_line = [0.0]
		self.opti_data = [0.0]

		#subscribe to inputs and sensor in formation topics
		rospy.Subscriber("data_to_store_" + str(car_number), Float32MultiArray, self.callback_data_line)



	
	
	
	def run_game_pad(self):
		rate = rospy.Rate(10) # 10hz
		date_time = datetime.datetime.now()
		date_time_str = date_time.strftime("%m_%d_%Y_%H_%M_%S")
		#date_time = date_time.strftime("%H_%M_%S")
		subfolder = '/Data_recordings/'
		current_dir = os.path.realpath(os.path.dirname(__file__))
		file_name = '/home/lorenzo/OneDrive/PhD/ubuntu_stuff/python_world/MPCC_Lyons/Data_recording_GP' + 'recording_' + date_time_str + '.csv'
		start_clock_time = rospy.get_rostime()

		with open(file_name, 'w+') as file:
			writer = csv.writer(file)
			writer.writerow(['opti x', 'opti y', 'opti theta', 'opti time', 'safety_value', 'throttle', 'steering', 'current', 'voltage', 'IMU[0]', 'IMU[1]', 'IMU[2]', 'velocity','elapsed time'])
			data_msg = Float32MultiArray()
			while not rospy.is_shutdown():
				#store data here
				stop_clock_time = rospy.get_rostime()
				elapsed_time = stop_clock_time.secs - start_clock_time.secs + (stop_clock_time.nsecs - start_clock_time.nsecs)/1000000000
				data_line = [self.Jet_data_line[0], self.Jet_data_line[1], self.Jet_data_line[2], self.Jet_data_line[3], self.Jet_data_line[4], self.Jet_data_line[5], self.Jet_data_line[6], self.Jet_data_line[7], self.Jet_data_line[8], self.Jet_data_line[9]]
				writer.writerow(data_line)
				print(data_line)
				data_msg.data = data_line
				self.pub_data_line.publish(data_msg)
				rate.sleep()
				
		
		#file.close()

		

	#Safety callback function
	def callback_data_line(self, Jet_data_line):
		self.Jet_data_line = Jet_data_line.data

	#Throttle callback function
	def callback_throttle(self, opti_data):
		self.opti_data = opti_data.data




if __name__ == '__main__':
	try:
		car_number = 3
		recording = record_input_and_sensor_data(car_number)
		recording.run_game_pad()
	except rospy.ROSInterruptException:
		pass
