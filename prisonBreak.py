#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

class Robot:

	def __init__(self):
		self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
		rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
		self.state = 'forward'
		self.time = rospy.get_time()
		
	def bumped(self, msg):
		self.state = 'backward'
		self.time = rospy.get_time()

	def run(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			if self.state == 'forward':
				twist.linear.x = 0.2
				twist.angular.z = 0
			elif self.state == 'backward':
				twist.linear.x = -0.2
				twist.angular.z = 0
				if (rospy.get_time() - self.time) > 2:
					self.time = rospy.get_time() 
					self.state = 'turn'
			elif self.state == 'turn':
				twist.linear.x = 0
				twist.angular.z = 2
				if (rospy.get_time() - self.time) > 4:
					twist.angular.z = 0
					self.state = 'forward'
			self.pub.publish(twist)
			rate.sleep()
			
rospy.init_node('prison_break')
robot = Robot()
robot.run()
