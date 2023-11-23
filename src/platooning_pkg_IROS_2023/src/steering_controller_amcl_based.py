#!/usr/bin/env python


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
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from functions_for_steering_controller import find_s_of_closest_point_on_global_path, produce_track,produce_marker_array_rviz
from visualization_msgs.msg import MarkerArray, Marker


class steering_controller_class:
	def __init__(self, car_number,rate):
		

		#set up variables
		self.car_number = car_number

		self.rate = rate
		self.kp = 1

		# initialize state variables
		# [x y theta]
		self.state = [0, 0, 0]
		self.t_prev = 0.0
		self.previous_index = 0 # initial index for closest point in global path

		# initiate steering variables
		self.steering_command_prev = 0

		# set up publisher
		self.steering_publisher = rospy.Publisher('steering_' + str(car_number), Float32, queue_size=1)
        	self.rviz_global_path_publisher = rospy.Publisher('rviz_global_path_' + str(self.car_number), MarkerArray, queue_size=10)

		#subscribers
		self.global_pose_subscriber = rospy.Subscriber('odom_' + str(car_number), Odometry, self.odometry_callback)

	def odometry_callback(self,odometry_msg):
		quaternion = (
		    odometry_msg.pose.pose.orientation.x,
		    odometry_msg.pose.pose.orientation.y,
		    odometry_msg.pose.pose.orientation.z,
		    odometry_msg.pose.pose.orientation.w)
		euler = euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		self.state = [odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, yaw]



	def generate_track(self, track_choice):
		# choice = 'savoiardo'
		# choice = 'straight_line'

		# number of checkpoints to be used to define each spline of the track
		n_checkpoints = 1000

		# cartesian coordinates (x-y) of the points defining the track and the gates
		Checkpoints_x, Checkpoints_y = produce_track(track_choice, n_checkpoints)

		# associate these points to the global path x-y-z variables
		self.x_vals_global_path = Checkpoints_x
		self.y_vals_global_path = Checkpoints_y

		# from the x-y-z points obtain the sum of the arc lenghts between each point, i.e. the path seen as a function of s
		spline_discretization = len(Checkpoints_x)
		self.s_vals_global_path = np.zeros(spline_discretization)
		for i in range(1,spline_discretization):
		    self.s_vals_global_path[i] = self.s_vals_global_path[i-1] + np.sqrt((self.x_vals_global_path[i]-self.x_vals_global_path[i-1])**2+
				                                                        (self.y_vals_global_path[i]-self.y_vals_global_path[i-1])**2)

		# s_vals_global_path = sum of the arc lenght along the original path, which are going to be used to re-parametrize the path

		# generate splines x(s) and y(s) where s is now the arc length value (starting from 0)
		#self.x_of_s = interpolate.CubicSpline(self.s_vals_global_path, self.x_vals_global_path)
		#self.y_of_s = interpolate.CubicSpline(self.s_vals_global_path, self.y_vals_global_path)

		# produce and send out global path message to rviz, which contains information about the track (i.e. the global path)
		rgba = [219.0, 0.0, 204.0, 0.6]
		marker_type = 4
		self.global_path_message = produce_marker_array_rviz(self.x_vals_global_path, self.y_vals_global_path, rgba, marker_type)
		self.rviz_global_path_publisher.publish(self.global_path_message)



	def compute_steering_control_action(self):

		# just a guess for how far the robot has travelled along the path
		estimated_ds = 0.5
		# measure the closest point on the global path, retturning the respective s parameter and its index
		s, self.current_index = find_s_of_closest_point_on_global_path(np.array([self.state[0], self.state[1]]), self.s_vals_global_path,
		                                                          self.x_vals_global_path, self.y_vals_global_path,
		                                                          self.previous_index, estimated_ds)
		
		# update index along the path to know where to search in next iteration
		self.previous_index = self.current_index
		print(s)






if __name__ == '__main__':
	try:
		car_number = os.environ["car_number"]
		rospy.init_node('steering_control_node_' + str(car_number), anonymous=False)
		rate = rospy.Rate(10) #Hz
		global_path_message_rate = 50 # publish 1 every 50 control loops

		#set up steering controller
		vehicle_controller = steering_controller_class(car_number, rate)
		# straight_line
		# savoiardo
		vehicle_controller.generate_track('straight_line')
		
		counter = 0

		while not rospy.is_shutdown():
			#run steering control loop
			vehicle_controller.compute_steering_control_action()

			# this is just to republish global path message every now and then
			if counter > global_path_message_rate:
				vehicle_controller.rviz_global_path_publisher.publish(vehicle_controller.global_path_message)
				counter = 0 # reset counter

			#update counter
			counter = counter + 1
			rate.sleep()



	except rospy.ROSInterruptException:
		pass
