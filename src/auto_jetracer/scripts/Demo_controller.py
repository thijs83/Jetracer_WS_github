#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from serial_read import Readline
import time
import serial
import numpy as np


class Demo_controller:
	
	def __init__(self, car_number, leader_number):
		#Setup node and topics subscription
		print("setup ros topics and node")
		#Car number
		self.car_number = car_number

		#initialize variables
		self.tag_point = [1.0, 1.0]
		self.lidar_point = [1.0, 1.0]
		self.leader_throttle = 0.13
		self.Kp_dist = 0.1
		self.ref_dist = 0.5

		rospy.init_node('Demo_controller' + str(self.car_number), anonymous=True)
		rospy.Subscriber("tag_point_shifted"+str(self.car_number), PointStamped, self.callback_tag_point, queue_size=1)
		rospy.Subscriber("cluster_point"+str(self.car_number), PointStamped, self.callback_lidar_point, queue_size=1)
		rospy.Subscriber("throttle_"+str(leader_number), Float32, self.callback_leader_throttle, queue_size=1)
		rospy.Subscriber("ref_dist", Float32, self.callback_ref_dist, queue_size=1)
		rospy.Subscriber("Kp_dist", Float32, self.callback_Kp_dist, queue_size=1)

		self.pub_throttle = rospy.Publisher("throttle_" + str(self.car_number), Float32, queue_size=1)
		self.pub_steering = rospy.Publisher("steering_" + str(self.car_number), Float32, queue_size=1)
	
		
		
		
		
	def execute_controller(self):
		rate = rospy.Rate(10)	#frequency of controller	
		print("starting the controller")
		# Run the while loop for the controller
		old_dist = 0
		Kd_dist = 6
		while not rospy.is_shutdown():
			#write controller here
			
			#Pure pursuite trying to reach the point the leading car is at right now
			x_point = 0.5 * (self.tag_point[0] + self.lidar_point[0])
			y_point = 0.5 * (self.tag_point[1] + self.lidar_point[1])
			dist = np.sqrt(x_point ** 2 + y_point ** 2)
			alpha = np.arctan(y_point / x_point)
			steering = -np.sign(alpha)*np.arctan(0.175 * 2 * np.cos(0.5*np.pi-np.abs(alpha))/ (3 * dist))
			#convert radians to [-1, 1] for steering commands
			max_steer_deg = 17
			steering_command = (steering/np.pi*180)/max_steer_deg

			if steering_command < -0.5:
				steering_command = -0.5
			elif steering_command > 0.5:
				steering_command = 0.5

			self.pub_steering.publish(steering_command)

			#throttle control
			throttle_command = self.leader_throttle + (dist - self.ref_dist) * self.Kp_dist + Kd_dist * (dist - old_dist)
			old_dist = np.sqrt(x_point ** 2 + y_point ** 2) # not using dist to avoid pointer business
			if throttle_command < 0.1:
				throttle_command = 0.1
			elif throttle_command > 0.17:
				throttle_command = 0.17

			self.pub_throttle.publish(throttle_command)


			# Sleep for the time set in the rate
			rate.sleep()




	#tag_point callback function
	def callback_tag_point(self,tag_point):
		self.tag_point = [tag_point.point.x, tag_point.point.y]

	#Reference velocity callback function
	def callback_lidar_point(self,lidar_point):
		self.lidar_point = [lidar_point.point.x, lidar_point.point.y]
		
	#Reference velocity callback function
	def callback_leader_throttle(self,leader_throttle):
		self.leader_throttle = leader_throttle.data		
		
	#Reference velocity callback function
	def callback_ref_dist(self, ref_dist):
		self.ref_dist = ref_dist.data
		print('ref_dist set to: ', ref_dist)			
		
	#Reference velocity callback function
	def callback_Kp_dist(self, Kp_dist):
		self.Kp_dist = Kp_dist.data
		print('Kp_dist set to: ', Kp_dist)


		

if __name__ == '__main__':
	print("Starting pid-controller for velocity")
	try:	
		car_number = 2
		leader_number = 1
		Demo_controller = Demo_controller(car_number, leader_number)
		Demo_controller.execute_controller()
	except rospy.ROSInterruptException:
		driving.car.throttle = 0
