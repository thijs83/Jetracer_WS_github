#!/usr/bin/env python3

import rospy
import pygame
import time
import os
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray, Bool


class teleop_gamepad:
	def __init__(self):
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
		self.V_target = 1.0

		#saturate acceleration limits
		self.acc_sat = 1

		#Setup topics publishing and nodes
		self.throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=8)
		self.acc_publisher = rospy.Publisher('acceleration_' + str(car_number), Float32, queue_size=1)

		pub_steering = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=8)
		pub_add_mpc = rospy.Publisher('add_mpc_gamepad', Bool, queue_size=8)
		pub_Kp_dist = rospy.Publisher('Kp_dist_gamepad', Float32, queue_size=8)

		self.gains_subscriber = rospy.Subscriber('linear_controller_gains', Float32MultiArray, self.gains_callback)
		self.v_encoder_subscriber = rospy.Subscriber('velocity_' + str(car_number), Float32, self.sub_vel_callback)
		self.v_target_subscriber = rospy.Subscriber('platoon_speed', Float32, self.v_target_callback)
		# initialize encoder reading
		self.velocity = 0

		# also publishing safety value
		pub_safety_value = rospy.Publisher('safety_value', Float32, queue_size=8)

		rospy.init_node('teleop_gamepad' + str(car_number), anonymous=True)
		self.rate = rospy.Rate(10) # 10hz

		while not rospy.is_shutdown():
			pygame.event.pump()

			#Obtain gamepad values
			throttle_joystick = -j.get_axis(1) #Left thumbstick Y
			steering = j.get_axis(2) #Right thumbstick X
			#print("Throttle_3:", throttle)
			#print("Steering_3:", steering)

			u_lin = self.h*(self.velocity - self.V_target)
			u_lin = self.saturate_acc(u_lin * throttle_joystick)
			print('u_lin = ', u_lin)
			self.acc_publisher.publish(Float32(u_lin)) #publish acceleration for followers

			tau = self.acc_2_throttle(u_lin)
			self.publish_throttle(tau) # so you can turn on and off from velocity following from joystick
			

			#Pubblish gamepad values
			#pub_throttle.publish(throttle * 0.14)  #reduce the throttle to keep velocity rasonable
			pub_steering.publish(steering* 0.1+0.05) 	#reduce the steering to keep going straight-ish

			#safety value publishing
			if j.get_button(7) == 1:
				print('safety off')
				pub_safety_value.publish(1)
			else:
				pub_safety_value.publish(0)
				
			for event in pygame.event.get(): # User did something.
				if event.type == pygame.JOYBUTTONDOWN:
					if j.get_button(4) == 1:
						print("add mpc set to true")

						pub_add_mpc.publish(True)
					if j.get_button(0) == 1:
						print("add mpc set to false")

						pub_add_mpc.publish(False)
					if j.get_button(1) == 1:
						Kp_dist = Kp_dist + 0.05
						print("reference distance set to:", Kp_dist)

						pub_Kp_dist.publish(Kp_dist)
					if j.get_button(3) == 1:
						Kp_dist = Kp_dist - 0.05
						print("Kp_dist set to:", Kp_dist)

						pub_Kp_dist.publish(Kp_dist)

			self.rate.sleep()






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
		tau = (acc + C * (self.velocity - 1))/a_th + 0.13

		#riccardo's magic formula
		#c1 = 90
		#c5 = 1.36
		#c6 = 0.2028
		#c3 = 0.87 #*0.6
		#c4 = 61.2
		#tau = (np.tan((acc+c3*self.velocity+c4)*np.pi/(c1*c5)-np.pi/2))/c6

		return tau



	def saturate_acc(self,acc):
		if acc >= self.acc_sat:
			acc = self.acc_sat
		elif acc <= -self.acc_sat:
			acc = -self.acc_sat

		return acc
							
			
	def publish_throttle(self, tau):
		# saturation limits for tau
		if tau < 0.104:
			tau = 0
		elif tau > 1:
			tau = 1

		throttle_val = Float32(tau)

		# for data storage purpouses
		#self.throttle = throttle_val.data

		#publish inputs
		self.throttle_publisher.publish(throttle_val)						
						
					
					
									

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass
