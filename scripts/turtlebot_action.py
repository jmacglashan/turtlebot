#!/usr/bin/python

import threading
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import math



class TurtlebotAction:

        def __init__(self):
                self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
                rospy.init_node('turtlebot_action', anonymous=True)
                rospy.Subscriber('/burlap_action', String, self.burlapCallback, queue_size=1)

                self.rotateSpeed = -0.9
                self.forwardSpeed = 0.15
                self.smoothness = 0.4

                self.rotateControlTime = 2.75
                self.forwardControlTime = 1.75
                
                self.isExecuting = False

                #set predefined twists
                self.rotateTwist = Twist()
                self.rotateTwist.angular.z = self.rotateSpeed

		self.rotateCCWTwist = Twist()
		self.rotateCCWTwist.angular.z = -self.rotateSpeed

                self.forwardTwist = Twist()
                self.forwardTwist.linear.x = self.forwardSpeed

		self.next = None
		self.lastAction = None

                rospy.spin()


        def burlapCallback(self, data):

		print 'getting callback', data.data

                if not self.isExecuting:
                        self.isExecuting = True
                        actionName = data.data
			self.command(actionName)
		else:
			self.next = data.data

	def command(self, actionName):
		timer = None
                if actionName == 'rotate':
			self.lastAction = actionName
                	timer = rospy.Timer(rospy.Duration(self.smoothness), self.rotPub)
                        threading.Timer(self.rotateControlTime, self.cancelTimer, [timer]).start()
                elif actionName == 'rotate_ccw':
			self.lastAction = actionName
                        timer = rospy.Timer(rospy.Duration(self.smoothness), self.rotCCWPub)
                        threading.Timer(self.rotateControlTime, self.cancelTimer, [timer]).start()
                elif actionName == 'forward':
			self.lastAction = actionName
                        timer = rospy.Timer(rospy.Duration(self.smoothness), self.forwardPub)
                        threading.Timer(self.forwardControlTime, self.cancelTimer, [timer]).start()

                else:   
                        print 'unknown action', actionName


        def rotPub(self, event):
                self.pub.publish(self.rotateTwist)

	def rotCCWPub(self, event):
		self.pub.publish(self.rotateCCWTwist)

        def forwardPub(self, event):
                self.pub.publish(self.forwardTwist)

	def cancelTimer(self, timer):
		if self.next is None:
			timer.shutdown()
			self.isExecuting = False
		else:
			nc = self.next	
			self.next = None
			if self.lastAction != nc:
				timer.shutdown()
				self.command(nc)
			else:
				threading.Timer(self.rotateControlTime, self.cancelTimer, [timer]).start()
			

if __name__ == '__main__':
        TurtlebotAction()
