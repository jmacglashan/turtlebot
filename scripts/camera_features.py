#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

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

			print 'converring color space'
			conv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCR_CB)
			print 'getting dist image'

			y, cr, cb = cv2.split(conv)
			cv2.imwrite('y.jpg', y)
			cv2.imwrite('cr.jpg', cr)
			cv2.imwrite('cb.jpg', cb)

			di = self.distImage(conv, (100, 255, 64))
			print 'writing dist image'
			cv2.imwrite('bdist.jpg', di)
			cv2.imwrite('extract.jpg', cv_image)

			#cv2.imwrite('extract.jpg', cv_image)
			#cv2.imwrite('extractconv.jpg', conv)
			
			self.nPrints -= 1
			print 'finished writing'

	def distImage(self, img, targetCol):
		height, width, channels = img.shape
		tImg = np.zeros((height,width,3), np.int)
		tImg[:,:] = targetCol
		sImg = img.astype(np.int) - tImg
		sqImg = np.square(sImg)
		y, cr, cb = cv2.split(sqImg)
		ssqImg = y + cr + cb
		sqrtImg = np.sqrt(ssqImg)
		dImg = sqrtImg.astype(np.uint8)
		

		return dImg

	



if __name__ == '__main__':
	CameraFeatures()
