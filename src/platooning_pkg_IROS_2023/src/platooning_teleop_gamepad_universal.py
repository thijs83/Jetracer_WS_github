#!/usr/bin/env python3

import rospy
import pygame
import time
import os
from std_msgs.msg import Float32, Float32MultiArray



def teleop_gamepad():
	#Initialize pygame and gamepad
	pygame.init()
	j = pygame.joystick.Joystick(0)
	j.init()
	print ('Initialized Joystick : %s' % j.get_name())
	print('remove safety by pressing R1 button')

	car_number = os.environ["car_number"]
	ref_dist = 0.5
	Kp_dist = 0.1

	self.h = -1.0

	#Setup topics publishing and nodes
	self.throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=8)
	pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=8)
	pub_ref_dist = rospy.Publisher('ref_dist_gamepad', Float32, queue_size=8)
	pub_Kp_dist = rospy.Publisher('Kp_dist_gamepad', Float32, queue_size=8)

	self.gains_subscriber = rospy.Subscriber('linear_controller_gains', Float32MultiArray, self.gains_callback)
	self.v_encoder_subscriber = rospy.Subscriber('velocity_' + str(car_number), Float32, self.sub_vel_callback)
	self.v_target_subscriber = rospy.Subscriber('platoon_speed', Float32, self.v_target_callback)
	# initialize encoder reading
	self.velocity = 0

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

		u_lin = self.h*(self.velocity - self.V_target)
		print('u_lin = ', u_lin)
		tau = self.acc_2_throttle(u_lin)
		self.publish_throttle(tau)
		

		#Pubblish gamepad values
		pub_throttle.publish(throttle * 0.14)  #reduce the throttle to keep velocity rasonable
		pub_steering.publish(steering* 0.33) 	#reduce the steering to keep going straight-ish

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
					#print("ciao")
					pub_ref_dist.publish(ref_dist)
				if j.get_button(0) == 1:
					ref_dist = ref_dist - 0.05
					print("reference distance set to:", ref_dist)
					#print("buongiorno")
					pub_ref_dist.publish(ref_dist)
				if j.get_button(1) == 1:
					Kp_dist = Kp_dist + 0.05
					print("reference distance set to:", Kp_dist)
					#print("buonasera")
					pub_Kp_dist.publish(Kp_dist)
				if j.get_button(3) == 1:
					Kp_dist = Kp_dist - 0.05
					print("Kp_dist set to:", Kp_dist)
					#print("buonanotte")
					pub_Kp_dist.publish(Kp_dist)


	def v_target_callback(self,v_target_msg):
		self.V_target = v_target_msg.data

	def sub_vel_callback(self, msg):
		# velocity from encoder
		self.velocity = msg.data

	def gains_callback(self, gains):
		# leader only has h to follow
		#self.kp = gains.data[2]
		#self.kd = gains.data[1]
		self.h = gains.data[0]	

	def acc_2_throttle(self, acc):
		# compute inverted dynamics to recover throttle from required acceleration
		C = 1.54 / 1.63  # longitudinal damping coefficient divided by the mass
		a_th = 60 / 1.63  # motor curve coefficient divided by the mass
		b_th = 1.54 / 1.63

		# xdot4 = -C * (x[3] - 1) + (u[0] - 0.129) * a_th
		tau = (acc + C * (self.velocity - 1))/a_th + 0.129
		return tau							
			
	def publish_throttle(self, tau):
		# saturation limits for tau
		if tau < 0:
			tau = 0
		elif tau > 1:
			tau = 1

		throttle_val = Float32(tau)

		# for data storage purpouses
		self.throttle = throttle_val.data

		#publish inputs
		self.throttle_publisher.publish(throttle_val)						
						
					
					
									




		rate.sleep()

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass
