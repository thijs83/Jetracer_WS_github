#!/usr/bin/env python3
import numpy as np
import os
import sys
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from datetime import datetime
import csv
import rospkg
from custom_msgs_optitrack.msg import custom_opti_pose_stamped_msg
from geometry_msgs.msg import PointStamped



class Platooning_controller_class:
	def __init__(self, car_number):
		rospy.init_node('Platooning_control_node_' + str(car_number), anonymous=False)

		#set up variables
		self.car_number = car_number

		# setu up N
		self.N = 30  # must match the number of stages in the solver
		# generate parameters
		self.V_target = 1  # in terms of being scaled down the proportion is vreal life[km/h] = v[m/s]*42.0000  (assuming the 30cm jetracer is a 3.5 m long car)
		self.dt = 0.1  # so first number is the prediction horizon in seconds -this is the dt of the solver so it will think that the control inputs are changed every dt seconds
		self.kp = -1.0
		self.kd = -2.0
		self.h = -1.0
		self.d_safety = 0.5

		# initialize state variables
		# [v v_rel x_rel]
		self.state = [0, 0, 0]
		self.t_prev = 0.0

		# initiate steering variables
		self.steering_command_prev = 0
		self.tau_filter = 1/(2*3.14)
		self.a = self.dt/(self.dt+self.tau_filter)
		self.tag_point = [1.0, 1.0]
		self.lidar_point = [1.0, 1.0]



		# set up publisher and subscribers
		self.throttle_publisher = rospy.Publisher('throttle_' + str(car_number), Float32, queue_size=1)
		self.steering_publisher = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=1)
		self.safety_value_subscriber = rospy.Subscriber('safety_value', Float32, self.safety_value_subscriber_callback)
		self.gains_subscriber = rospy.Subscriber('linear_controller_gains', Float32MultiArray, self.gains_callback)
		self.v_target_subscriber = rospy.Subscriber('platoon_speed', Float32, self.v_target_callback)
		self.safety_distance = rospy.Subscriber('d_safety', Float32, self.d_safety_callback)

		#self.occupancy_xyr_4_subscriber = rospy.Subscriber('occupancy_xyr_4', Float32MultiArray, self.occupancy_xyr_4_subscriber_callback)
		self.v_encoder_subscriber = rospy.Subscriber('velocity_' + str(car_number), Float32, self.sub_vel_callback)
		self.x_rel_subscriber = rospy.Subscriber('distance_' + str(car_number), Float32, self.distance_subscriber_callback) #subscribe to lidar and camera data output

		rospy.Subscriber("tag_point_shifted_"+str(self.car_number), PointStamped, self.callback_tag_point, queue_size=1)
		rospy.Subscriber("cluster_point_"+str(self.car_number), PointStamped, self.callback_lidar_point, queue_size=1)



	def start_platooning_control_loop(self):
		self.rate = rospy.Rate(1 / self.dt)
		while not rospy.is_shutdown():
			# Throttle control
			#compute linear controller contorl action 
			# state = [v v_rel x_rel]

			u_lin = self.kd * self.state[1] + self.kp*(-self.state[2]+self.d_safety) + self.h*(self.state[0] - self.V_target)
			print('u_lin = ', u_lin)
			tau = self.acc_2_throttle(u_lin)
			self.publish_throttle(tau)

			# Steering control
						#Pure pursuite trying to reach the point the leading car is at right now
			x_point = 0.5 * (self.tag_point[0] + self.lidar_point[0])
			y_point = 0.5 * (self.tag_point[1] + self.lidar_point[1])
			dist = np.sqrt(x_point ** 2 + y_point ** 2)
			alpha = np.arctan(y_point / x_point)
			steering = -np.sign(alpha)*np.arctan(0.175 * 2 * np.cos(0.5*np.pi-np.abs(alpha))/ (0.5 + dist))
			#convert radians to [-1, 1] for steering commands
			max_steer_deg = 17
			steering_command = (steering/np.pi*180)/max_steer_deg

			if steering_command < -1:
				steering_command = -1
			elif steering_command > 1:
				steering_command = 1

			#applying first order filter
			steering_command_out = (1-self.a)*steering_command+self.a*self.steering_command_prev
			#update prev
			self.steering_command_prev = steering_command_out 
			

			self.steering_publisher.publish(steering_command_out)


			self.rate.sleep()






	def acc_2_throttle(self, acc):
		# compute inverted dynamics to recover throttle from required acceleration
		C = 1.54 / 1.63  # longitudinal damping coefficient divided by the mass
		a_th = 60 / 1.63  # motor curve coefficient divided by the mass
		b_th = 1.54 / 1.63

		# xdot4 = -C * (x[3] - 1) + (u[0] - 0.129) * a_th
		tau = (acc + C * (self.state[0] - 1))/a_th + 0.129
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



	def sub_vel_callback(self, msg):
		# state = [v v_rel x_rel]
		self.state[0] = msg.data

	def distance_subscriber_callback(self, msg):
		# state = [v v_rel x_rel]
		now = rospy.get_rostime()
		t = now.secs + now.nsecs/10**9
		dt = t-self.t_prev
		self.t_prev = t
		self.state[1] = -(msg.data-self.state[2])/dt
		self.state[2] = msg.data
		#compute x_rel and v_rel

	def safety_value_subscriber_callback(self, msg):
		print(msg.data)
		self.safety_value = msg.data

	#tag_point callback function
	def callback_tag_point(self,tag_point):
		self.tag_point = [tag_point.point.x, tag_point.point.y]

	#lidar point callback function
	def callback_lidar_point(self,lidar_point):
		self.lidar_point = [lidar_point.point.x, lidar_point.point.y]

	def gains_callback(self, gains):
		self.kp = gains.data[2]
		self.kd = gains.data[1]
		self.h = gains.data[0]

	def v_target_callback(self,v_target_msg):
		self.V_target = v_target_msg.data

	def d_safety_callback(self,d_safety_callback_msg):
		self.d_safety = d_safety_callback.data


	def occupancy_xyr_4_subscriber_callback(self, msg):
		# some legacy shorter vector can also be accepted
		# temporary block out to just test the tracking performance
		dyn_ob_traj = msg.data
		if len(dyn_ob_traj) == 45:
			self.dyn_ob_traj_x[0:15] = dyn_ob_traj[0:15]
			self.dyn_ob_traj_y[0:15] = dyn_ob_traj[15:30]
			self.dyn_ob_traj_r[0:15] = dyn_ob_traj[30:45]
			self.dyn_ob_traj_x[15:31] = dyn_ob_traj[14] + np.zeros(15)
			self.dyn_ob_traj_y[15:31] = dyn_ob_traj[29] + np.zeros(15)
			self.dyn_ob_traj_r[15:31] = dyn_ob_traj[34] + np.zeros(15)
		elif len(dyn_ob_traj) == 90:
			self.dyn_ob_traj_x[0:31] = dyn_ob_traj[0:30]
			self.dyn_ob_traj_y[0:31] = dyn_ob_traj[30:60]
			self.dyn_ob_traj_r[0:31] = dyn_ob_traj[60:90]





if __name__ == '__main__':
    try:

        car_number = os.environ["car_number"]

        # choose solver to run
        vehicle_controller = Platooning_controller_class(car_number)
        vehicle_controller.start_platooning_control_loop()




    except rospy.ROSInterruptException:
        pass
