#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from burlap_msgs.msg import timed_action

import math



class TurtlebotAction:

        def __init__(self):
                self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
                rospy.init_node('turtlebot_action', anonymous=True)
                rospy.Subscriber('/burlap_action_timed', timed_action, self.burlapCallback, queue_size=1)

                self.rotateSpeed = -0.9
                self.forwardSpeed = 0.15
                self.smoothness = 0.4

                self.rotateControlTime = 2.75
                self.forwardControlTime = 1.75
                
                self.isExecuting = False
		self.lastActionTime = rospy.Time.now()
		self.firstAction = True

                #set predefined twists
                self.rotateTwist = Twist()
                self.rotateTwist.angular.z = self.rotateSpeed

		self.rotateCCWTwist = Twist()
		self.rotateCCWTwist.angular.z = -self.rotateSpeed

                self.forwardTwist = Twist()
                self.forwardTwist.linear.x = self.forwardSpeed

                rospy.spin()


        def burlapCallback(self, data):
		
		timeDiff = data.header.stamp - self.lastActionTime
		
		if timeDiff > rospy.Duration(0) or self.firstAction:
			
			if not self.isExecuting:
		
				self.isExecuting = True

                        	actionName = data.data

	                        timer = None
        	                if actionName == 'rotate':
                	                timer = rospy.Timer(rospy.Duration(self.smoothness), self.rotPub)
                        	        rospy.sleep(self.rotateControlTime)
				elif actionName == 'rotate_ccw':
					timer = rospy.Timer(rospy.Duration(self.smoothness), self.rotCCWPub)
	                                rospy.sleep(self.rotateControlTime)
        	                elif actionName == 'forward':
                	                timer = rospy.Timer(rospy.Duration(self.smoothness), self.forwardPub)
                        	        rospy.sleep(self.forwardControlTime)

	                        else:   	
        	                        print 'unknown action', actionName

                	        if timer != None:
                        	        timer.shutdown()
			
				self.isExecuting = False

				self.lastActionTime = rospy.Time.now()
				self.firstAction = False

	

        def rotPub(self, event):
                self.pub.publish(self.rotateTwist)

	def rotCCWPub(self, event):
		self.pub.publish(self.rotateCCWTwist)

        def forwardPub(self, event):
                self.pub.publish(self.forwardTwist)


if __name__ == '__main__':
        TurtlebotAction()
