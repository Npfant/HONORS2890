#!/usr/bin/env python3

import roslib
roslib.load_manifest('robot')
import rospy
import cv2
import math
from sensor_msgs.msg import Image, LaserScan
from robot.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

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
        #print(output)
        return output

class Dodgeball:

	def __init__(self):
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
		#rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
		rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.measure)
		rospy.Subscriber('/odom', Odometry, self.handle_pose)
		
		self.state = 'kick'
		self.time = rospy.get_time()
		
		self.bearing, self.distance, self.x, self.y = -1, -1, -1, -1
		self.angle, self.range, self.theta = -1, -1, 0
		self.xint, self.yint, self.diff = -1, -1, 1
		self.bearingpid = PID(320, .01, 0, 0.01, 1)
		self.distancepid = PID(1.5, -0.35, 0, 0.02, 0.25)
		self.rangepid = PID(0, -0.35, 0, 0.02, 0.25)
		self.anglepid = PID(0, 0.48, 0, 0.01, 1)

	def handle_pose(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(_, _, self.theta) = euler_from_quaternion(q)
		
	def get_vector(self, from_x, from_y, to_x, to_y):
		bearing = math.atan2(to_y - from_y, to_x - from_x)
		distance = math.sqrt((from_x - to_x)**2 + (from_y - to_y)**2)
		return (bearing, distance)
		
	def vibeCheck(self, actual, desired):
		diff = actual - desired
		if (diff) > math.pi:
			real_diff = diff % (-2 * math.pi)
		elif (diff) < -math.pi:
			real_diff = diff % (2 * math.pi)
		else:
			real_diff = diff
		return (real_diff)

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
				if(self.bearing > 300 and self.bearing < 340 and self.distance > 1.4 and self.distance < 1.6):
					self.xint = self.x + 2.12 * math.cos(self.theta - math.pi/4)
					self.yint = self.y + 2.12 * math.sin(self.theta - math.pi/4)
					self.xgoal = self.x + 3 * math.cos(self.theta)
					self.ygoal = self.y + 3 * math.sin(self.theta)
					self.state = 'navToInt'
				if(self.bearing > 0 and self.distance > 0):
					twist.angular.z = self.bearingpid.get_output(self.bearing)
					twist.linear.x = self.distancepid.get_output(self.distance)

			if self.state == 'navToInt':
				twist.linear.x = 0
				twist.angular.z = 0
				self.angle, self.range = self.get_vector(self.x, self.y, self.xint, self.yint)
				self.diff = self.vibeCheck(self.theta, self.angle)
				twist.angular.z = self.anglepid.get_output(self.diff)
				twist.linear.x = self.rangepid.get_output(self.range)
				if(abs(self.angle) < 0.75 and abs(self.range) < 0.1):
					self.state = 'navToKick'
				print(self.xint,self.yint)
				
			if self.state == 'navToKick':
				twist.linear.x = 0
				twist.angular.z = 0
				self.angle, self.range = self.get_vector(self.x, self.y, self.xgoal, self.ygoal)
				self.diff = self.vibeCheck(self.theta, self.angle)
				twist.angular.z = self.anglepid.get_output(self.diff)
				twist.linear.x = self.rangepid.get_output(self.range)
				if(abs(self.angle) < 0.75 and abs(self.range) < 0.1):
					self.state = 'lineup'
				print(self.xgoal,self.ygoal)
				
			if self.state == 'lineup':
				twist.linear.x = 0
				twist.angular.z = .5
				if(self.bearing > 310 and self.bearing < 330):
					self.time = rospy.get_time()
					self.state = 'kick'
				if(self.bearing > 0):
					twist.angular.z = self.bearingpid.get_output(self.bearing)
					
			if self.state == 'kick':
				twist.angular.z = 0
				twist.linear.x = 1
				if(rospy.get_time() - self.time) > 2:
					self.state = 'search'		
						
			print(self.state)
			#print(self.bearing)
			#print(self.distance)
			#print(self.x, self.y, self.theta)
			#print(self.angle, self.range)			
			

			self.pub.publish(twist)
			rate.sleep()

rospy.init_node('Dodgeball')
dodgeball = Dodgeball()
dodgeball.start()
