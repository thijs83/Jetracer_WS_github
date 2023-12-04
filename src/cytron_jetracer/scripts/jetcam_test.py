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
		camera = CSICamera(width=1280, height=720, capture_device=0, fps=120)
		print('set up CSI camera')

		#save an image
		image = camera.read()
		#image_compressed = bgr8_to_jpeg(image)
		image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		filename = 'image_grayscale.png'
		cv2.imwrite(filename, image_gray)	


		camera.running = True   
		camera.observe(self.update_image, names='value')
		self.bridge = CvBridge()

		# set up ros node and publisher
		self.pub_image = rospy.Publisher("camera_" + str(car_number) + "/image_raw", Image, queue_size=1)

		



	def update_image(self,change):
		image = change['new']
		#image_compressed = bgr8_to_jpeg(image)
 
		try:	
			# the encoding is 8UC3, you can check this by doing rostopic echo camera_1/encoding
			image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # COLOR_BGR2GRAY   COLOR_RGB2BGR

			self.image_message = self.bridge.cv2_to_imgmsg(image_gray, encoding="passthrough")
		except CvBridgeError as e:
			print(e)

		# publish image
		
		self.pub_image.publish(self.image_message)





if __name__ == '__main__':

	#Setup node and topics subscription
	car_number = os.environ["car_number"]
	rospy.init_node('jetcam_node_'+str(car_number), anonymous=True)
	print(" setup jetcam node " +str(car_number))

	try:
		jetcam_node_obj = jetcam_node(car_number)
	except rospy.ROSInterruptException:
		print('failed to launch jetcam node '+str(car_number))
