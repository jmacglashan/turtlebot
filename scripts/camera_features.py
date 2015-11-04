#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraFeatures:

	def __init__(self):
		self.nPrints = 1
		self.bridge = CvBridge()
		rospy.init_node('camera_features', anonymous=True)
		rospy.Subscriber('camera/rgb/image_color', Image, self.cameraCallback, queue_size=1)
		
		rospy.spin()
		

	def cameraCallback(self, data):
		if self.nPrints > 0:
			print 'getting callback'
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
			except CvBridge, e:
				print e

			conv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCR_CB)

			cv2.imwrite('extract.jpg', cv_image)
			cv2.imwrite('extractconv.jpg', conv)
			self.nPrints -= 1
			print 'finished writing'


if __name__ == '__main__':
	CameraFeatures()
