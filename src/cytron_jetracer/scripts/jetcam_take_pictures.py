#!/usr/bin/env python3

import rospy
from jetcam.csi_camera import CSICamera
import time
import os
#from jetcam.usb_camera import USBCamera
#import ipywidgets
#from IPython.display import display
from jetcam.utils import bgr8_to_jpeg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from jetcam.utils import bgr8_to_jpeg
from getkey import getkey
import rospkg
from pynput.keyboard import Key, Listener

# the integrated camera is called:  imx219-160

# useful commands to check that the camera is connected (execute from terminal)
# v4l2-ctl --list-devices              (imx219 is the CSI camera)
# v4l2-ctl -d /dev/video0 --list-formats-ext          (gives some info on the camera)
# nvgstcapture-1.0      (this should start streaming the camera feed) NOTE! if it cannot 
			#connect to CSI camera it ould be because some previous	session 
			#is running for some reason in the background. Run the following command and try again
#sudo service nvargus-daemon restart

# to visualize image feed from terminal (to check if all is good) use:  NOTE it only works with grayscale images
# rosrun image_view image_vieimage:=/camera_1/image_raw _do_dynamic_scaling:=true

# usefull tutorial:    http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython





class jetcam_node:

	def __init__(self,car_number):

		# set up CSI camera using jetcam
		self.camera = CSICamera(width=1280, height=720, capture_device=0, fps=120)
		print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
		print('succesfully set up CSI camera')

		self.pictures_taken = 0
		rospack = rospkg.RosPack()
		self.path_to_folder = rospack.get_path('cytron_jetracer') + '/scripts/Images_4_camera_calibration'
		print("press f to take a picture, saved in folder:")
		print(self.path_to_folder)

		# Window name in which image is displayed 
		self.window_name = 'image'
		  
 

		
	def start_taking_picture(self):
		#Print contol hints
		print('press the space key to save a picture, press esc to quit')
		#rate = rospy.Rate(5) # 10hz

		# Collect events until released
		#with Listener(on_press=self.on_press) as listener:
		#    listener.join()
		while True:
			image = self.camera.read()
			#self.image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert to grayscale since camera calibration wants grayscale anyway

			#cv2.imshow('camera', self.image_gray)
			print(cv2.waitKey(0))
			if cv2.waitKey(1) == 27: 
				break  # esc to quit
		cv2.destroyAllWindows()



				

				
	def on_press(self,key):
		#save an image
		print('{0} pressed'.format(key))
		if key == key.space:
			image = self.camera.read()
			self.image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert to grayscale since camera calibration wants grayscale anyway

			# Using cv2.imshow() method 
			# Displaying the image 
			cv2.imshow(self.window_name, self.image_gray)
			#cv2.waitKey(0) 



		elif key == 's':
			filename = self.path_to_folder + '/image_grayscale_' + str(self.pictures_taken) +'.png'
			cv2.imwrite(filename, self.image_gray)
			self.pictures_taken = self.pictures_taken + 1
			print('taken picture n. ' + str (self.pictures_taken))


		elif key == key.esc:
			# Stop listener
			# closing all open windows 
			cv2.destroyAllWindows() 
			return False





if __name__ == '__main__':

	#Setup node and topics subscription
	car_number = os.environ["car_number"]
	rospy.init_node('jetcam_save_image_node_'+str(car_number), anonymous=True)
	print(" setup jetcam image saver node " + str(car_number))

	try:
		jetcam_node_obj = jetcam_node(car_number)
		jetcam_node_obj.start_taking_picture()
	except rospy.ROSInterruptException:
		print('failed to launch jetcam node '+str(car_number))


























