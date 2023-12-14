#!/usr/bin/env python3

import rospy
import math
import os
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32,Float64MultiArray







class racecar:
	def __init__(self, car_number, steering_gain, steering_offset, throttle_gain):
		print("Starting racecar" + str(car_number))
		#Base features from cytron - initialize car variables and tune settings
		self.car = NvidiaRacecar()
		self.car.steering_gain = steering_gain	#do not change this value
		self.car.steering_offset = steering_offset
		self.car.throttle_gain = throttle_gain


		# set inputs to 0 when starting up
		self.car.steering = 0.0
		self.car.throttle = 0.0

		# additional features by Lyons
		self.car_number = float(car_number)
		self.safety_value = 0

		#set up ros nodes for this vehicle
		print("setting ros topics and node")
		rospy.init_node('racecar_' + str(car_number), anonymous=True)
		rospy.Subscriber('steering_' + str(car_number), Float32, self.callback_steering)
		rospy.Subscriber('throttle_' + str(car_number), Float32, self.callback_throttle)
		rospy.Subscriber('safety_value', Float32, self.callback_safety)
		self.commands_timestamped = rospy.Publisher("commands_timestamped_car_" + str(car_number), Float64MultiArray, queue_size=1)
		rospy.spin()

	#Safety callback function
	def callback_safety(self, safety_val_incoming):
		self.safety_value = safety_val_incoming.data
	
	#Steering control callback function
	def callback_steering(self, steer):
		self.car.steering = steer.data
		#rospy.loginfo("Steering " + str(self.car_number) + ": %s", str(steer.data))
		#publish command inputs with a timestamp
		time_now = rospy.get_rostime()
		time_float = time_now.secs + (time_now.nsecs)/1000000000
		commands_data_msg = Float64MultiArray()
		commands_data_msg.data = [time_float, self.car.throttle, self.car.steering]
		self.commands_timestamped.publish(commands_data_msg)
		
	
	#Throttle callback function
	def callback_throttle(self, throttle):
		if self.safety_value == 1:
			self.car.throttle = throttle.data
		else:
			self.car.throttle = 0
			#rospy.loginfo("Throttle" + str(self.car_number) + ": %s", str(self.car.throttle*self.safety_value))

		#publish command inputs with a timestamp
		time_now = rospy.get_rostime()
		time_float = time_now.secs + (time_now.nsecs)/1000000000
		commands_data_msg = Float64MultiArray()
		commands_data_msg.data = [time_float, self.car.throttle, self.car.steering]
		
		self.commands_timestamped.publish(commands_data_msg)
	





if __name__ == '__main__':
	car_number = os.environ["car_number"]
	print('car_number = ', os.environ["car_number"])
	if float(car_number) == 1:
		steering_gain = -0.6
		steering_offset = 0.0  
		print('setting steer gain and offset for car number 1')
	elif float(car_number) == 2:
		steering_gain = -0.5
		steering_offset = 0.0
		print('setting steer gain and offset for car number 2')

	elif float(car_number) == 3:
		steering_gain = -0.4
		steering_offset = 0.05
		print('setting steer gain and offset for car number 3')

	else:
		steering_gain = -0.4
		steering_offset = 0.0008


	throttle_gain = 1
	racecar(car_number, steering_gain, steering_offset, throttle_gain)





