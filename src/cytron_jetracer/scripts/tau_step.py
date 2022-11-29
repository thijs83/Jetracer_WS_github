#!/usr/bin/env python3

import rospy
import pygame
import time
from std_msgs.msg import Float32

#Initialize pygame and gamepad
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print('Initialized Joystick ; %s' % j.get_name())

def teleop_gamepad():

	car_number = 3
	#ref_tau = 0.10
	incr = 0.005
	throttle_pub = 0
	scale = 0.1
	# Set up topics publishing and nodes
	pub_throttle = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=8)
	pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=8)
	# also publishing safety value
	pub_safety_value = rospy.Publisher('safety_value', Float32, queue_size=8)

	rospy.init_node('teleop_gamepad' + str(car_number), anonymous=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		pygame.event.pump()

		steering = j.get_axis(2) #Right thumbstick X
		throttle = -j.get_axis(1) #Left thumbstick X

		pub_throttle.publish(scale*throttle)
		rospy.loginfo(scale)
		pub_steering.publish(steering)
		rospy.loginfo(steering)

		if j.get_button(7) == 1:
			print('safety off')
			pub_safety_value.publish(1)
		else:
			pub_safety_value.publish(0)
			
		for event in pygame.event.get(): # User did something.
			if event.type == pygame.JOYBUTTONDOWN: # button Y
				if j.get_button(4) == 1:
					scale = scale + incr
					#throttle_pub = throttle*scale
					print('tau_ref = ', scale)
				if j.get_button(0) == 1: # button A
					scale = scale - incr
					#throttle_pub = throttle*scale
					print('tau_ref = ', scale)
				if j.get_button(1) == 1: # button B
					scale = 0
					#throttle_pub = scale*throttle
					print('tau_ref = ', scale)
				if j.get_button(3) == 1: # button X
					scale = 1
					#throttle_pub = scale*throttle
					print('tau_ref = ', scale)
		if throttle_pub > 1:
			throttle_pub = 1
		if throttle_pub < 0:
			throttle_pub = 0

		rate.sleep()

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass



