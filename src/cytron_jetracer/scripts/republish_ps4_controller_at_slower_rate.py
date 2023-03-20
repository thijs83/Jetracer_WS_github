#!/usr/bin/env python3

import rospy
import math
import os
from std_msgs.msg import Float32





class republisher:

	def __init__(self, car_number,freq):
		self.car_number = car_number
		self.safety = 0.0
		self.throttle = 0.0
		self.steering = 0.0
		# Setup topics publishing and nodes
		self.pub_throttle = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=1)
		self.pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=1)
		rospy.Subscriber('joystick_steering_' + str(car_number), Float32, self.callback_steering)
		rospy.Subscriber('joystick_throttle_' + str(car_number), Float32, self.callback_throttle)
		rospy.init_node('teleop_gamepad_republisher' + str(car_number), anonymous=True)

		print('running republilshing at Hz = ', str(freq))
		rate = rospy.Rate(freq) 
		while not rospy.is_shutdown():
			#Publish gamepad values
			self.pub_throttle.publish(self.throttle)
			self.pub_steering.publish(self.steering)
			rate.sleep()



	#Steering control callback function
	def callback_steering(self, steer):
		self.steering = steer.data

	def callback_throttle(self, throttle):
		self.throttle = throttle.data





if __name__ == '__main__':
	try:
		car_number = os.environ["car_number"]
		print('car_number = ', os.environ["car_number"])
		#car_number = 3
		freq = 10 # republishing frequency
		controller = republisher(car_number, freq)


	except rospy.ROSInterruptException:
		pass





