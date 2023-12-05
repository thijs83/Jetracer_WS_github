#!/usr/bin/env python3


import numpy as np
import os
import sys
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
import rospkg
from functions_for_controllers import evaluate_Fx_2
from scipy import optimize


class leader_longitudinal_controller_class:
	def __init__(self, car_number):
		

		#set up variables
		self.car_number = car_number

		# set controller gains
		self.v_ref = 1 #[m/s]
		self.kp = 0.2

		# initialize state variables
		self.sensors = [0.0, 0.0, 0.0, 0.0, 0.0, .0, 0.0]
		self.v = 0.0

		# initiate steering variables
		self.steering_command_prev = 0

		# set up publisher
		self.throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=1)

		#subscribers
		self.v_encoder_subscriber = rospy.Subscriber('sensors_' + str(car_number), Float32MultiArray, self.sensors_callback)

		# set up feed forward action (if the target velocity changes it needs to be recomputed)
		func = lambda th : evaluate_Fx_2(self.v_ref, th)
		self.tau_ff = optimize.fsolve (func, 0.14)[0]
		print ('feed forward action =', self.tau_ff)


	def sensors_callback(self, msg):
		self.sensors = np.array(msg.data)
		self.v = self.sensors[6]


	def compute_longitudinal_control_action(self):
		# evaluate control action as a feed forward part plus a feed back part
		tau_fb = - self.kp * (self.v-self.v_ref)
		# apply saturation to feedbacck part
		tau_fb = np.min([0.05,tau_fb])
		tau_fb = np.max([-0.05,tau_fb])

		# sum the two contributions
		tau = self.tau_ff + tau_fb

		# apply saturation to overall control action
		tau = np.min([1,tau])
		tau = np.max([-1,tau])
		#publish command
		self.throttle_publisher.publish(tau)






if __name__ == '__main__':
	try:
		car_number = os.environ["car_number"]
		rospy.init_node('longitudinal_control_node_' + str(car_number), anonymous=False)
		rate = rospy.Rate(10) #Hz

		#set up longitudinal controller
		vehicle_controller = leader_longitudinal_controller_class(car_number)

		while not rospy.is_shutdown():
			#run longitudinal controller loop
			vehicle_controller.compute_longitudinal_control_action()
			rate.sleep()



	except rospy.ROSInterruptException:
		pass
