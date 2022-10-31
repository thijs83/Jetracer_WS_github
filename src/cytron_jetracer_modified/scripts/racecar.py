#!/usr/bin/env python3

import rospy
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32
from serial_read import Readline
import time
import serial

class racecar:
	
	def __init__(self):
		#Initialize car variable and tune settings
		print("setup nvidea racecar")
		self.car = NvidiaRacecar()
		self.car.steering_gain = 0.35	#do not change this value
		self.car.steering_offset = 0.00
		self.car.throttle_gain = 1
		self.car.steering = 0.0
		self.car.throttle = 0.0
		
		#Setup node and topics subscription
		print("setup ros topics and node")
		rospy.init_node('racecar', anonymous=True)
		rospy.Subscriber("steering1", Float32, self.callback_steering)
		rospy.Subscriber("throttle1", Float32, self.callback_throttle)
		pub = rospy.Publisher("velocity1", Float32, queue_size=1)

		
		
		#Start serial connection
		try:
			print("Setting up USB connection")
			arduino = serial.Serial(port="/dev/arduino", baudrate=115200, timeout=1)
			print("Setting up USB connection done")
			Rline = Readline(arduino)
		except:
			print('Please check the USB port connections')
		
		#Setup store variables PID
		self.error_prev = 0
		self.tprev = time.time()
		self.I = 0
		
		#Setup reference velocity for the pid controller and initial velocity
		self.VelReference = 0
		self.Vel = 0
		
		rate = rospy.Rate(100)
		#Run the while loop for the controller
		print("starting the controller")
		data_prev = 0
		print(self.Vel)
		while not rospy.is_shutdown():
			
			#Check for new serial data
			data = Rline.readline()
			if data:
				data = float(data)
				if data != data_prev:
					self.Vel = data/1000
					data_prev = data
				pub.publish(self.Vel)
				
			
			# Sleep for the time set in the rate
			rate.sleep()
			
		#if note is sut down set input zero
		self.car.throttle = 0
		
		
		
	#Steering control callback function
	def callback_steering(self,steer):
		self.car.steering = steer.data
		rospy.loginfo("Steering: %s", str(steer.data))

	#Steering control callback function
	def callback_throttle(self,throttle):
		self.car.throttle = throttle.data
		rospy.loginfo("Throttle:: %s", str(throttle.data))
		
		

		
		



if __name__ == '__main__':
	print("Starting racecar")
	try:
		driving = racecar()
	except rospy.ROSInterruptException:
		driving.car.throttle = 0
