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
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from functions_for_controllers import find_s_of_closest_point_on_global_path, produce_track,produce_marker_array_rviz,produce_marker_rviz,set_up_topology
from visualization_msgs.msg import MarkerArray, Marker


class relative_state_publisher:
	# this class sets up a node that publishes the relative position and velocity between 2 vehicles. This is needed to run the longitudinal controller.
	def __init__(self, car_number):
		

		#set up variables
		self.car_number = car_number
		self.leader_number = set_up_topology(car_number)


		# initialize state variables
		# [x y theta]
		self.state = [0, 0, 0]
		self.leader_position = [0, 0]
		self.previous_path_index = 0 # initial index for closest point in global path
		self.sensors = [0.0, 0.0, 0.0, 0.0, 0.0, .0, 0.0]
		self.v = 0.0
		self.v_leader = 0.0


		# set up publisher
		self.relative_state_publisher = rospy.Publisher('relative_state_' + str(self.car_number), Float32MultiArray, queue_size=10)

		#subscribers
		self.rviz_closest_point_on_path_leader = rospy.Subscriber('rviz_closest_point_on_path_' + str(self.leader_number), Marker, self.leader_path_progress_callback)
		self.state_subscriber = rospy.Subscriber('sensors_' + str(self.car_number), Float32MultiArray, self.sensors_callback)
		self.leader_state_subscriber = rospy.Subscriber('sensors_' + str(self.leader_number), Float32MultiArray, self.sensors_leader_callback)

		self.tf_listener = tf.TransformListener()




	def sensors_callback(self, msg):
		sensors = np.array(msg.data)
		self.v = sensors[6]

	def sensors_leader_callback(self, msg):
		sensors = np.array(msg.data)
		self.v_leader = sensors[6]

	def leader_path_progress_callback(self, msg):
		self.leader_position = [msg.pose.position.x,msg.pose.position.y]
		
		


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
		
		#this is now done by the steering controller... maybe merge this into a single file hmhm
		# produce and send out global path message to rviz, which contains information about the track (i.e. the global path)
		#rgba = [219.0, 0.0, 204.0, 0.6]
		#marker_type = 4
		#self.global_path_message = produce_marker_array_rviz(self.x_vals_global_path, self.y_vals_global_path, rgba, marker_type)
		#self.rviz_global_path_publisher.publish(self.global_path_message)




	def evaluate_relative_state(self):

		#get latest transform data for robot pose
		self.tf_listener.waitForTransform("/map", "/base_link_" + str(self.car_number), rospy.Time(), rospy.Duration(1.0))
		(robot_position,robot_quaternion) = self.tf_listener.lookupTransform("/map",  "/base_link_" + str(self.car_number), rospy.Time(0))
		# transform from quaternion to euler angles
		#robot_euler = euler_from_quaternion(robot_quaternion)
		#robot_theta = robot_euler[2]

		# measure the closest point on the global path, retturning the respective s parameter and its index
		estimated_ds = 1 # just a first guess for how far the robot has travelled along the path
		s, self.current_path_index = find_s_of_closest_point_on_global_path(np.array([robot_position[0], robot_position[1]]), self.s_vals_global_path,
		                                                          self.x_vals_global_path, self.y_vals_global_path,
		                                                          self.previous_path_index, estimated_ds)

		
		# update index along the path to know where to search in next iteration
		self.previous_path_index = self.current_path_index
		x_closest_point = self.x_vals_global_path[self.current_path_index]
		y_closest_point = self.y_vals_global_path[self.current_path_index]


		# evaluate relative position between ego vehicle and leader
		distance = np.sqrt((x_closest_point-self.leader_position[0])**2+(y_closest_point-self.leader_position[1])**2 + 0.001)
		
		#evalaute relative velocity betwwen ego vehicle and leader
		rel_vel = self.v-self.v_leader


		#publish relative state
		floatarray_msg = Float32MultiArray()
		floatarray_msg.data = [rel_vel, -distance]
		self.relative_state_publisher.publish(floatarray_msg)
		#print('relative state =', [rel_vel, distance])









if __name__ == '__main__':
	try:
		car_number = os.environ["car_number"]
		rospy.init_node('relative_state_publisher_node_' + str(car_number), anonymous=False)
		rate = rospy.Rate(10) #Hz

		#set up relative distance evaluator. Note that is measures the distance between the projections on the global path
		relative_state_publisher_obj = relative_state_publisher(car_number)
		# straight_line
		# savoiardo
		# straight_line_my_house
		relative_state_publisher_obj.generate_track('straight_line_my_house')
		

		while not rospy.is_shutdown():
			#run evalaution in a loop
			try:
				relative_state_publisher_obj.evaluate_relative_state()
			except:
				print('failed to evaluate relative position, problay problem with tf')




	except rospy.ROSInterruptException:
		pass
