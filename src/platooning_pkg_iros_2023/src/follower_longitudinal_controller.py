#!/usr/bin/env python3
import numpy as np
import os
import sys
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from datetime import datetime
import csv
import rospkg
from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg
from geometry_msgs.msg import PointStamped
import random


from functions_for_controllers import set_up_topology, evaluate_Fx_2
from scipy import optimize

class follower_longitudinal_controller_class:
	def __init__(self, car_number):

		#set up variables
		self.car_number = car_number
		self.leader_number = set_up_topology(car_number)

		# setu up N
		#self.N = 30  # must match the number of stages in the solver
		# generate parameters
		self.v_ref = 1  # in terms of being scaled down the proportion is vreal life[km/h] = v[m/s]*42.0000  (assuming the 30cm jetracer is a 3.5 m long car)
		self.dt = 0.1  # so first number is the prediction horizon in seconds -this is the dt of the solver so it will think that the control inputs are changed every dt seconds
		#self.kp = -1.0
		#self.kd = -2.0
		#self.h = -0.5
		#self.d_safety = 0.5
		self.acc_sat = 1
		self.kp = -(self.acc_sat/(2*self.v_ref))
		self.h = 2*self.kp
		self.kd = -2.0*np.sqrt(-self.kp)
		self.d_safety = -self.acc_sat/self.kp
		self.add_mpc = False

		

		# initialize state variables
		# [v v_rel x_rel]
		#self.state = [0, 0, 0]
		#self.t_prev = 0.0
		#self.t_prev_encoder = 0.0
		self.relative_state = [0.0,0.0]
		self.v = 0.0


		# initiate steering variables
		#self.steering_command_prev = 0
		#self.tau_filter = 1/(2*3.14)
		#self.a = self.dt/(self.dt+self.tau_filter)
		#self.tag_point = [1.0, 1.0]
		#self.lidar_point = [1.0, 1.0]
		#self.acc_leader = 0
		#self.acc_leader_prev = 0
		#self.vel_leader_prev = 0
		#self.u_mpc_prev = 0 # just to filter the random noise to lower frequency
		#self.acc_leader_encoder = 0.0
		#self.acc_leader_prev_encoder = 0.0






		# set up publisher and subscribers
		self.throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=1)

		# controller parameters subscribers
		self.safety_value_subscriber = rospy.Subscriber('safety_value', Float32, self.safety_value_subscriber_callback)
		self.gains_subscriber = rospy.Subscriber('linear_controller_gains', Float32MultiArray, self.gains_callback)
		self.v_ref_subscriber = rospy.Subscriber('platoon_speed', Float32, self.v_ref_callback)
		self.safety_distance = rospy.Subscriber('d_safety', Float32, self.d_safety_callback)
		self.add_mpc_subscriber = rospy.Subscriber('add_mpc_gamepad', Bool, self.add_mpc_callback)
		

		#self.acc_leader_subscriber = rospy.Subscriber('acceleration_' + str(self.leader_number), Float32, self.acc_leader_callback)
		# state subscribers
		self.sensors_subscriber = rospy.Subscriber('sensors_' + str(car_number), Float32MultiArray, self.sensors_callback)
		self.relative_state_subscriber = rospy.Subscriber('relative_state_' + str(car_number), Float32MultiArray, self.relative_state_callback)

		#leader_encoder_topic = 'rel_theta' + str(self.leader_number)
		#print(leader_encoder_topic)

		#self.acc_leader_subscriber_encoder = rospy.Subscriber(leader_encoder_topic, Float32, self.acc_leader_callback_encoder)
		#rospy.Subscriber("tag_point_shifted_"+str(self.car_number), PointStamped, self.callback_tag_point, queue_size=1)
		#rospy.Subscriber("cluster_point_"+str(self.car_number), PointStamped, self.callback_lidar_point, queue_size=1)

		#self.u_control_publisher = rospy.Publisher('u_control_' + str(car_number), Float32, queue_size=1)


	# controller parameter callbacks

	def safety_value_subscriber_callback(self, msg):
		#print(msg.data)
		self.safety_value = msg.data

	def gains_callback(self, gains):
		self.kp = gains.data[2]
		self.kd = gains.data[1]
		self.h = gains.data[0]

	def v_ref_callback(self,v_ref_msg):
		self.v_ref = v_ref_msg.data

	def d_safety_callback(self,d_safety_callback_msg):
		self.d_safety = d_safety_callback.data

	def add_mpc_callback(self,add_mpc_msg):
		if float(self.car_number) == 1 and float(self.car_number) == 2:
			self.add_mpc = False
		else:
			self.add_mpc = add_mpc_msg.data



	# state callbacks

	def sensors_callback(self, msg):
		self.sensors = np.array(msg.data)
		self.v = self.sensors[6]

	def relative_state_callback(self, msg):
		# [rel_vel, distance]
		self.relative_state = np.array(msg.data)









	def publish_throttle(self, tau):
		# saturation limits for tau
		if tau < 0:
			tau = 0
		elif tau > 1:
			tau = 1

		throttle_val = Float32(tau)

		# for data storage purpouses
		#self.throttle = throttle_val.data

		#publish inputs
		self.throttle_publisher.publish(throttle_val)



	# def sub_vel_callback(self, msg):
	# 	# state = [v v_rel x_rel]
	# 	self.state[0] = msg.data

	# def distance_subscriber_callback(self, msg):
	# 	# state = [v v_rel x_rel]
	# 	now = rospy.get_rostime()
	# 	t = now.secs + now.nsecs/10**9
	# 	dt = t-self.t_prev
	# 	self.t_prev = t
	# 	self.state[1] = (-msg.data+self.d_safety-self.state[2])/dt
	# 	self.state[2] = -msg.data+self.d_safety
	# 	#compute x_rel and v_rel



	# #tag_point callback function
	# def callback_tag_point(self,tag_point):
	# 	self.tag_point = [tag_point.point.x, tag_point.point.y]

	# #lidar point callback function
	# def callback_lidar_point(self,lidar_point):
	# 	self.lidar_point = [lidar_point.point.x, lidar_point.point.y]







	# def acc_leader_callback(self,acc_msg):
	# 	tau_filter = 1/(2*np.pi*1)
	# 	c = self.dt/(self.dt+tau_filter)
	# 	self.acc_leader = (1-c) * acc_msg.data + c * self.acc_leader_prev
	# 	self.acc_leader_prev = self.acc_leader

	# def acc_leader_callback_encoder(self,velocity_msg):
	# 	now = rospy.get_rostime()
	# 	t = now.secs + now.nsecs/10**9
	# 	dt = t-self.t_prev_encoder
	# 	self.t_prev_encoder = t

	# 	acc_encoder_new = (velocity_msg.data-self.vel_leader_prev)/dt
	# 	self.vel_leader_prev = velocity_msg.data

	# 	tau_filter = 1/(2*np.pi*1)
	# 	#c = self.dt/(self.dt+tau_filter)
	# 	c=0.0

	# 	self.acc_leader_encoder = (1-c) * acc_encoder_new + c * self.acc_leader_prev_encoder
	# 	self.acc_leader_prev_encoder = self.acc_leader_encoder
		






	def compute_longitudinal_control_action(self):

		# evaluate linear controller action
		# [rel_vel, distance]
		u_lin = self.kd * self.relative_state[0] + self.kp* self.relative_state[1] + self.h*(self.v - self.v_ref)

		
		
		#u_mpc = self.generete_mpc_action(u_lin)
		#u_control = u_lin+u_mpc

		# artificial saturation bounds
		u_control = self.saturate_acc(u_lin)


		#self.u_control_publisher.publish(float(u_control))

		#self.acc_publisher.publish(Float32(u_control)) # advertise acceleration for follower
		tau = self.acc_2_throttle(u_control)
		self.publish_throttle(tau)

		# # Steering control
		# kd_omega = 0.5

		# #Pure pursuite trying to reach the point the leading car is at right now
		# #x_point = 0.5 * (self.tag_point[0] + self.lidar_point[0])
		# #y_point = 0.5 * (self.tag_point[1] + self.lidar_point[1])

		# x_point = self.lidar_point[0]
		# y_point = self.lidar_point[1]
		# #dist = np.sqrt(x_point ** 2 + y_point ** 2)  # we have a subscriber for that already
		# alpha = np.arctan(y_point / (x_point+0.001))
		# steering = -np.sign(alpha)*np.arctan(0.175 * 2 * np.cos(0.5*np.pi-np.abs(alpha))/ (2 + self.state[2])) #
		


		# #steering = -np.sign(alpha)*np.arctan(0.175 / (self.v_ref) * kd_omega * np.abs(alpha))
		# #steering =  kd_omega*(1-np.cos(alpha))**0.5 * np.sign(-np.sin(alpha))

		# #convert radians to [-1, 1] for steering commands
		# max_steer_deg = 17
		# steering_command = (steering/np.pi*180)/max_steer_deg
		# steering_sat = 0.1
		# if steering_command < -steering_sat:
		# 	steering_command = -steering_sat
		# elif steering_command > steering_sat:
		# 	steering_command = steering_sat

		# steering_offset = 0.0 #-0.1
		# steering_command = steering_command + steering_offset

		# #applying first order filter
		# #steering_command_out = (1-self.a)*steering_command+self.a*self.steering_command_prev


		# #update prev
		# self.steering_command_prev = steering_command 
		

		# self.steering_publisher.publish(float(steering_command))








	def acc_2_throttle(self, acc):
		# evaluate control action based by solving the inverse dynamics
		func = lambda th : evaluate_Fx_2(self.v_ref, th) - acc
		tau = optimize.fsolve (func, 0.14)[0]
		return tau



		

	def generete_mpc_action(self, u_linear):
		if self.add_mpc:
			# evaluate new relative state using leader acceleration info
			x_dot_rel_k_plus_1 = self.state[1] + u_linear*self.dt - self.acc_leader_encoder*self.dt # - self.acc_leader*self.dt #
			x_rel_k_plus_1 = self.state[2] + self.state[1]*self.dt

			# for mpc line generation
			no_dist_kd = self.kd+self.h
			y_max = self.acc_sat/(-self.kp)*0.8 #last number is mpc line lowering coeff (1 is no lowering)
			mpc_slope = -(no_dist_kd)/(self.kp)
			x_line = (-y_max + x_rel_k_plus_1)/mpc_slope
			#print('y_max = ',y_max,' self.acc_leader_encoder = ', self.acc_leader_encoder,'x_rel_k_plus_1 =',x_rel_k_plus_1,'x_dot_rel_k_plus_1 = ',x_dot_rel_k_plus_1, "u_linear =", u_linear,'self.state[1]',self.state[1])

			#evaluate action
			u_mpc = (x_line - x_dot_rel_k_plus_1)/(self.dt*2)
			# corrupted mpc (filtered with noise to lower frequency)
			#u_mpc_new = self.acc_sat*(2*random.random()-1) # random number between amp*(-1 --> 1)
			c = 0.0
			u_mpc = (1-c) * u_mpc + c * self.u_mpc_prev
			self.u_mpc_prev = u_mpc


		else:
			u_mpc = 0.0

		print('u_mpc = ', u_mpc)
		return u_mpc
		
	def saturate_acc(self,acc):
		if acc >= self.acc_sat:
			acc = self.acc_sat
		elif acc <= -self.acc_sat:
			acc = -self.acc_sat

		return acc
	
	




if __name__ == '__main__':
	try:
		car_number = os.environ["car_number"]
		rospy.init_node('longitudinal_control_node_' + str(car_number), anonymous=False)
		rate = rospy.Rate(10) #Hz

		#set up longitudinal controller
		vehicle_controller = follower_longitudinal_controller_class(car_number)

		while not rospy.is_shutdown():
			#run longitudinal controller loop
			vehicle_controller.compute_longitudinal_control_action()
			rate.sleep()



	except rospy.ROSInterruptException:
		pass
