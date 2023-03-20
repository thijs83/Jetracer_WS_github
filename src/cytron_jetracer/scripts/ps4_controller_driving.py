#!/usr/bin/env python3
# FIRST OF ALL YOU MUST INSTALL the packet from here:
#https://pypi.org/project/pyPS4Controller/

import rospy
#import pygame
from pyPS4Controller.controller import Controller
import time
from std_msgs.msg import Float32
import os


class MyPS4Controller(Controller):

	def __init__(self, car_number,  **kwargs):
		Controller.__init__(self, **kwargs)
		self.car_number = car_number
		self.safety = 0.0
		self.throttle = 0.0
		self.steering = 0.0
		# Setup topics publishing and nodes
		self.pub_throttle = rospy.Publisher('joystick_throttle_' + str(car_number), Float32, queue_size=1)
		self.pub_steering = rospy.Publisher('joystick_steering_' + str(car_number), Float32, queue_size=1)
		# also publishing safety value
		self.pub_safety_value = rospy.Publisher('safety_value', Float32, queue_size=1)
		rospy.init_node('teleop_gamepad' + str(car_number), anonymous=True)



	def on_x_press(self):
		print("Hello world")
	def on_x_release(self):
		print("Goodbye world")

	def on_R1_press(self):
		print('safety off')
		self.safety = 1
		self.pub_safety_value.publish(1)
	def on_R1_release(self):
		print('safety on')
		self.safety = 0
		self.pub_safety_value.publish(0)

	def on_L3_up(self, value):
		self.throttle = -value/32767 * self.safety
		self.pub_throttle.publish(self.throttle)
	def on_L3_down(self, value):
		self.throttle = -value/32767 * self.safety
		self.pub_throttle.publish(self.throttle)
	def on_L3_y_at_rest(self):
		self.throttle = 0.0
		self.pub_throttle.publish(self.throttle)

	def on_L2_press(self, value):  # for some reason on the jetracer L3 gets taken as L2, who knows why
		self.steering = value/32767 * self.safety
		self.pub_steering.publish(self.steering)
		#print('steering =', self.steering)




if __name__ == '__main__':
	try:
		car_number = os.environ["car_number"]
		print('car_number = ', os.environ["car_number"])
		#car_number = 3
		controller = MyPS4Controller(car_number, interface="/dev/input/js0", connecting_using_ds4drv=False)
		controller.listen(timeout=60)
		# you can start listening before controller is paired, as long as you pair it within the timeout window

		

	except rospy.ROSInterruptException:
		pass
