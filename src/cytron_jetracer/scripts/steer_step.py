#!/usr/bin/env python3

import rospy
import pygame
import time
from std_msgs.msg import Float32

#Initialize pygame and gamepad
pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print ('Initialized Joystick : %s' % j.get_name())
print('remove safety by pressing R1 button')

def teleop_gamepad():


	car_number = 3
	ref_tau	 = 0.13  # Set the value able to steer the car at v = 1 m/s
	ref_steer = 0
	steering = ref_steer
	incr = 0.25
	#Setup topics publishing and nodes
	pub_throttle = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=8)
	pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=8)
	# also publishing safety value
	pub_safety_value = rospy.Publisher('safety_value', Float32, queue_size=8)

	rospy.init_node('teleop_gamepad' + str(car_number), anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		pygame.event.pump()

		#Obtain gamepad values
		throttle = ref_tau
		steering = steering

		#Pubblish gamepad values
		pub_throttle.publish(throttle)  #reduce the throttle to keep velocity rasonable
		rospy.loginfo(throttle)
		pub_steering.publish(steering)
		rospy.loginfo(steering)

		#safety value publishing
		if j.get_button(7) == 1:
			print('safety off')
			pub_safety_value.publish(1)
		else:
			pub_safety_value.publish(0)
			
		for event in pygame.event.get(): # User did something.
			if event.type == pygame.JOYBUTTONDOWN: # button Y
				if j.get_button(4) == 1:
					steering = steering + incr
					print('steer_ref = ', steering)
				if j.get_button(0) == 1: # button A
					steering = steering - incr
					print('steer_ref = ', steering)
				if j.get_button(1) == 1: # button B
					steering = ref_steer
					print('steer_ref = ', steering)
		if steering > 1:
			steering = 1
		if steering < -1:
			steering = -1

		rate.sleep()

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass
