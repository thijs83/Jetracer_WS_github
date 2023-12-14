#!/usr/bin/env python3

import rospy
import pygame
import time
from std_msgs.msg import Float32
import os

#Initialize pygame and gamepad
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print ('Initialized Joystick : %s' % j.get_name())
print('remove safety by pressing R1 button')

def teleop_gamepad(car_number):
	# this only controls the steering, the throttle can be controlled for example at a constant speed by running
	# leader controller in platooning_pkg_iros_2023

	#Setup topics publishing and nodes
	pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=8)
	pub_safety_value = rospy.Publisher('safety_value', Float32, queue_size=8)

	rospy.init_node('teleop_gamepad' + str(car_number), anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		pygame.event.pump()

		#Obtain gamepad values
		throttle = -j.get_axis(1) #Left thumbstick Y
		steering = j.get_axis(2) #Right thumbstick X
		#print("Throttle_3:", throttle)
		#print("Steering_3:", steering)

		#Pubblish gamepad values
		pub_steering.publish(steering)

		#safety value publishing
		if j.get_button(7) == 1:
			print('safety off')
			pub_safety_value.publish(1)
		else:
			pub_safety_value.publish(0)


		rate.sleep()

if __name__ == '__main__':
	try:
		car_number = os.environ['car_number']
		teleop_gamepad(car_number)
	except rospy.ROSInterruptException:
		pass