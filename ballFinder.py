#!/usr/bin/env python3

import roslib
roslib.load_manifest('robot')
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from robot.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError

class Detector:

	def __init__(self):
        # The image publisher is for debugging and figuring out
        # good color values to use for ball detection
		self.impub = rospy.Publisher('/ball_detector/image', Image, queue_size=1)
		self.locpub = rospy.Publisher('/ball_detector/ball_location', BallLocation,
                        		queue_size=1)
		self.bridge = CvBridge()
		self.bearing = -1
		self.distance = -1

		rospy.Subscriber('/camera/rgb/image_raw', Image, self.handle_image)
		rospy.Subscriber('/scan', LaserScan, self.handle_scan)

	def handle_image(self, msg):
		try:
			bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			image = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
		except (CvBridgeError, e):
			print ("e")
		(rows, columns, channels) = image.shape
        # Find the average column of the bright yellow pixels
        # and store as self.bearing. Store -1 if there are no
        # bright yellow pixels in the image. Yellow = (0, 255, 255) 160 190 190
		self.num = 0;
		self.sum = 0;
		
		for i in range (0, 639, 10):
			for j in range (50, 450, 3):
				if(image[j, i, 0] > 50 and image[j, i, 0] < 70):#blue/hue
					if(image[j, i, 1] > 50 and image[j, i, 1] < 70):#green/saturation
						if(image[j, i, 2] > 80 and image[j, i, 2] <100):#red/value
							self.sum += i
							self.num += 1
							image[j, i, 0] = 0
							image[j, i, 1] = 0
							image[j, i, 2] = 0
			
		if(self.sum > 0 and self.num >= 10):
			self.bearing = int(self.sum / self.num)
			image[:, self.bearing, 0] = 0
			image[:, self.bearing, 1] = 255
			image[:, self.bearing, 2] = 0
			image[:, 320, 0] = 0
			image[:, 320, 1] = 0
			image[:, 320, 2] = 0
		else: 
			self.bearing = -1
        # Feel free to change the values in the image variable
        # in order to see what is going on
        # Here we publish the modified image; it can be
        # examined by running image_view
		image2 = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
		self.impub.publish(self.bridge.cv2_to_imgmsg(image2, "bgr8"))
		cv2.destroyAllWindows()

	def handle_scan(self, msg):
        # If the bearing is valid, store the corresponding range
        # in self.distance. Decide what to do if range is NaN.
		if(self.bearing >= 0):
 			self.distance = msg.ranges[int(-self.bearing)]
 			if(self.distance != self.distance):
 				self.distance = -1
 			
		else:
			self.distance = -1
    
	def start(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			location = BallLocation()
			location.bearing = self.bearing
			location.distance = self.distance
			self.locpub.publish(location)
			rate.sleep()

rospy.init_node('ball_detector')
detector = Detector()
detector.start()
