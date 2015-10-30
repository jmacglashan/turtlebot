#!/usr/bin/python

import rospy
from std_msgs.msg import String
from burlap_msgs.msg import burlap_state
from burlap_msgs.msg import burlap_object
from burlap_msgs.msg import burlap_value
from geometry_msgs.msg import Pose

import math

class ParseMocap:

    def __init__(self, nBlocks):
	#self.minx = -1.
	self.minx = -1.16561865807
	#self.miny = -.9
	self.miny = -1.11893427372
	#self.maxx = 1.
	self.maxx = 0.908763885498
	#self.maxy = 1.1
	self.maxy = 0.916896104813

	self.twidth = -0.787930727005 - -1.13830828667
	self.theight = -0.762270748615 - -1.11893427372

	self.twidth /= 2.
	self.theight /= 2.
	
	self.north = norm((0.00263002398424, 0.000276191130979, -0.0616339966655, 0.998095333576))
	self.south = norm((0.00113216217142, -0.00203232187778, 0.994062721729, 0.108784668148))
	self.east = norm((0.00227167084813, 0.0051250760443, -0.760767698288, 0.649000167847))
	self.west = norm((-0.00304936780594, 0.00226421374828, 0.66129642725, 0.750115156174))

	print 'width height:', self.twidth, self.theight

	#print self.north
	#print self.south
	#print self.east
	#print self.west

	#self.resx = 11.
	#self.resy = 11.


	self.rangex = self.maxx - self.minx
	self.rangey = self.maxy - self.miny


	self.resx = self.rangex / self.twidth
	self.resy = self.rangey / self.theight

	print 'xres yres', self.resx, self.resy

        self.pub = rospy.Publisher('burlap_state', burlap_state, queue_size=1)
        rospy.init_node('mocap_parser', anonymous=True)
        rospy.Subscriber('/rigid_body_1/pose', Pose, self.turtlebotCallback)

	self.agent = None
	self.blocks = []
	for i in xrange(nBlocks):
		self.blocks.append(None)
		topic = '/rigid_body_%d/pose' % (i+2)
		print topic
		rospy.Subscriber(topic, Pose, self.blockCallback, callback_args=i)


        rospy.spin()

    def verifyStateReady(self):
	if self.agent == None:
		return False

	for el in self.blocks:
		if el == None:
			return False

	return True


    def publishState(self):
	if not self.verifyStateReady():
		return
	obArray = [self.agent]
	obArray += self.blocks

	s = burlap_state()
	s.objects = obArray

	self.pub.publish(s)
	

    def parsePose(self, pose):
	rx = pose.position.x
	ry = pose.position.y
	
	nx = (rx - self.minx) / self.rangex
	ny = (ry - self.miny) / self.rangey

	x = int(math.floor(nx*self.resx))
	y = int(math.floor(ny*self.resy))

	qx = pose.orientation.x
	qy = pose.orientation.y
	qz = pose.orientation.z
	qw = pose.orientation.w
	q = (qx, qy, qz, qw)
	dists = (quartDist(q, self.north), quartDist(q, self.south), quartDist(q, self.east), quartDist(q, self.west))
	#print q
	#print 'o_dist:', dists
	#print '----'

	dir = ''
	mi = dists.index(min(dists))
	if mi == 0:
		dir = 'north'
	elif mi == 1:
		dir = 'south'
	elif mi == 2:
		dir = 'east'
	elif mi == 3:
		dir = 'west'

	return (x, y, dir)


    def turtlebotCallback(self, data):
	botInfo = self.parsePose(data)
	xv = burlap_value()
        xv.attribute = 'x'
        xv.value = str(botInfo[0])

	yv = burlap_value()
        yv.attribute = 'y'
        yv.value = str(botInfo[1])

	dv = burlap_value()
	dv.attribute = 'direction'
	dv.value = botInfo[2]

	agent = burlap_object()
	agent.name = 'turtlebot'
	agent.object_class = 'agent'
	agent.values = [xv, yv, dv]

	self.agent = agent

	self.publishState()


    def blockCallback(self, data, args):
	blockInfo = self.parsePose(data)

	xv = burlap_value()
        xv.attribute = 'x'
        xv.value = str(blockInfo[0])

        yv = burlap_value()
        yv.attribute = 'y'
        yv.value = str(blockInfo[1])

	cv = burlap_value()
	cv.attribute = 'color'
	cv.value = 'yellow'
	
	sv = burlap_value()
	sv.attribute = 'shape'
	sv.value = 'chair'

	block = burlap_object()
	block.name = 'block' + str(args)
	block.object_class = 'block'
	block.values = [xv, yv, cv, sv]

	self.blocks[args] = block

	self.publishState()




def rot(x, theta):
	xp = x[0] * math.cos(theta) - x[1] * math.sin(theta)
	yp = x[0] * math.sin(theta) + x[1] * math.cos(theta)
	return (xp, yp)

def norm(q):
	sumsq = 0.
	for xi in q:
		sumsq += xi*xi
	mag = math.sqrt(sumsq)
	n = []
	for xi in q:
		n.append(xi / mag)
	return tuple(n)

def dot(a, b):
	sum = 0.
	for i in xrange(len(a)):
		sum += a[i] * b[i]
	return sum

def quartDist(a, b):
	dp = dot(a,b)
	return math.acos(2 * dp * dp - 1.) 


if __name__ == '__main__':
    ParseMocap(1)

