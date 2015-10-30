#!/usr/bin/python

import rospy
from std_msgs.msg import String
from burlap_msgs.msg import burlap_state
from burlap_msgs.msg import burlap_object
from burlap_msgs.msg import burlap_value
from turtlesim.msg import Pose

import math

class ParseTurtleSim:

	def __init__(self):
		self.pub = rospy.Publisher('burlap_state', burlap_state, queue_size=10)
		rospy.init_node('turtlesim_parser', anonymous=True)
		rospy.Subscriber('/turtle1/pose', Pose, self.turtlesimCallback)	
		
		self.agent = None

		rospy.spin()


	def turtlesimCallback(self, data):
		x = int(math.floor(data.x))
		y = int(math.floor(data.y))
		
		normAngle = data.theta / (2. * math.pi)
		if normAngle  < 0.:
			normAngle = 1. + normAngle

		dir = ''
		if normAngle < 1./8. or normAngle >= 7./8.:		
			dir = 'east'
		elif normAngle >= 1./8. and normAngle < 3./8.:
			dir = 'north'
		elif normAngle >= 3./8. and normAngle < 5./8.:
			dir = 'west'
		elif normAngle >= 5./8. and normAngle < 7./8.:
			dir = 'south'
		else:
			print 'error in angle:', data.theta, normAngle

		xv = burlap_value()
		xv.attribute = 'x'
		xv.value = str(x)

		yv = burlap_value()
		yv.attribute = 'y'
		yv.value = str(y)

		dirv = burlap_value()
		dirv.attribute = 'direction'
		dirv.value = dir

		agent = burlap_object()
		agent.name = 'turtle'
		agent.object_class = 'agent'
		agent.values = [xv, yv, dirv]
		
		self.agent = agent

		self.publishState()


	def publishState(self):
		if self.agent != None:
			s = burlap_state()
			s.objects = [self.agent]
			self.pub.publish(s)







if __name__ == '__main__':
	ParseTurtleSim()
