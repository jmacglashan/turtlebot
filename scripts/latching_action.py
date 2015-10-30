#!/usr/bin/python

import threading
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import math



class TurtlebotAction:

        def __init__(self):
                self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
                rospy.init_node('latching_action', anonymous=True)
                rospy.Subscriber('/latching_action', Twist, self.burlapCallback, queue_size=1)

		self.smoothness = 0.4
		
		self.nullTwist = Twist()
		self.twist = self.nullTwist
		
		timer = rospy.Timer(rospy.Duration(self.smoothness), self.pubTwist)

                rospy.spin()


        def burlapCallback(self, data):

		print 'getting callback', str(data)
		self.twist = data


	def pubTwist(self, event):
		self.pub.publish(self.twist)

if __name__ == '__main__':
        TurtlebotAction()
