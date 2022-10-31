#!/usr/bin/env python3

import rospy
import pygame
import time
from std_msgs.msg import Float32



def teleop_gamepad():
	#Initialize pygame and gamepad
	pygame.init()
	j = pygame.joystick.Joystick(0)
	j.init()
	print ('Initialized Joystick : %s' % j.get_name())
	print('remove safety by pressing R1 button')

	car_number = 1
	ref_dist = 0.5
	Kp_dist = 0.1
	#Setup topics publishing and nodes
	pub_throttle = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=8)
	pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=8)
	pub_ref_dist = rospy.Publisher('ref_dist', Float32, queue_size=8)
	pub_Kp_dist = rospy.Publisher('Kp_dist', Float32, queue_size=8)
	# also publishing safety value
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
		pub_throttle.publish(throttle * 0.14)  #reduce the throttle to keep velocity rasonable
		pub_steering.publish(steering)

		#safety value publishing
		if j.get_button(7) == 1:
			print('safety off')
			pub_safety_value.publish(1)
		else:
			pub_safety_value.publish(0)
			
		for event in pygame.event.get(): # User did something.
			if event.type == pygame.JOYBUTTONDOWN:
				if j.get_button(4) == 1:
					ref_dist = ref_dist + 0.05
					print("reference distance set to:", ref_dist)
					pub_ref_dist.publish(ref_dist)
				if j.get_button(0) == 1:
					ref_dist = ref_dist - 0.05
					print("reference distance set to:", ref_dist)
					pub_ref_dist.publish(ref_dist)
				if j.get_button(1) == 1:
					Kp_dist = Kp_dist + 0.05
					print("reference distance set to:", Kp_dist)
					pub_Kp_dist.publish(Kp_dist)
				if j.get_button(3) == 1:
					Kp_dist = Kp_dist - 0.05
					print("Kp_dist set to:", Kp_dist)
					pub_Kp_dist.publish(Kp_dist)										
					
					
					
									




		rate.sleep()

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass
