#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import math



class TurtleAction:

	def __init__(self):
		self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		rospy.init_node('turtle_action', anonymous=True)
		rospy.Subscriber('/burlap_action', String, self.burlapCallback)

		self.rotateSpeed = -9.
		self.forwardSpeed = 5.

		rospy.spin()


	def burlapCallback(self, data):
		actionName = data.data

		twistMsg = Twist()
		twistMsg.linear.x = 0.
		twistMsg.linear.y = 0.
		twistMsg.linear.z = 0.

		twistMsg.angular.x = 0.
                twistMsg.angular.y = 0.
                twistMsg.angular.z = 0.

		if actionName == 'rotate':
			twistMsg.angular.z = self.rotateSpeed

		elif actionName == 'forward':
			twistMsg.linear.x = self.forwardSpeed


		else:
			print 'unknown action', actionName

		self.pub.publish(twistMsg)



if __name__ == '__main__':
	TurtleAction()
