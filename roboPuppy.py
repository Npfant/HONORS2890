#!/usr/bin/env python3

import roslib
roslib.load_manifest('robot')
import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from robot.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

class PID:
    def __init__(self, goal, kp, ki, kd, max):
        self.goal = goal
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max = max
        self.previous_error = 0
        self.integral = 0
        
    def get_output(self, measurement):
        error = self.goal - measurement
        derivative = error - self.previous_error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        output = max(output, -self.max)
        output = min(output, self.max)
        self.previous_error = error
        #print(error)
        print(output)
        return output

class Puppy:

	def __init__(self):
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
		#rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
		rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.measure)
		
		self.state = 'search'
		self.time = rospy.get_time()
		
		self.bearing = -1
		self.distance = -1
		self.bearingpid = PID(320, .01, 0, 0.01, 1)
		self.distancepid = PID(1.5, -0.35, 0, 0.02, .25)
		
	#def bumped(self, msg):
		#self.state = 'backward'
		#self.time = rospy.get_time()
		
	def measure(self, msg):
		self.bearing = msg.bearing
		self.distance = msg.distance
		if(self.distance != self.distance):
			self.distance = -1
		
	def start(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			if self.state == 'search':
				twist.linear.x = 0
				twist.angular.z = .5
				if(self.bearing > 0 and self.distance > 0):
					self.state = 'approach'
			if self.state == 'approach':
				twist.linear.x = 0
				twist.angular.z = 0
				if(self.bearing < 0 or self.distance < 0):
					self.state = 'search' 
				if(self.bearing > 300 and self.bearing < 340 and self.distance > 						1.4 and self.distance < 1.6):
					self.state = 'kick'
					self.time = rospy.get_time()
				if(self.bearing > 0 and self.distance > 0):
					twist.angular.z = self.bearingpid.get_output(self.bearing)
					twist.linear.x = self.distancepid.get_output(self.distance)
				
			if self.state == 'kick':
				twist.linear.x = 1
				twist.angular.z = 0
				if(rospy.get_time() - self.time) > 2:
					self.state = 'search'
				
			#print(self.state)
			#print(self.bearing)
			print(self.distance)
			
			self.pub.publish(twist)
			rate.sleep()

rospy.init_node('roboPuppy')
puppy = Puppy()
puppy.start()
