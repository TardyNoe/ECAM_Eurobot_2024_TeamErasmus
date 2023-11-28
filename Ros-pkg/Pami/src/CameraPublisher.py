#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import std_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np

mtx1 = np.array([[899.9407098052037, 0.0, 935.8491370923285], [0.0, 900.2176117746629, 523.0356982676464], [0.0, 0.0, 1.0]])
dist1 = np.array([[-0.28804628309873526, 0.08216048075928635, 0.0005545901850384366, 0.00015312879975407738, -0.010170047195160393]])
h, w = (1080, 1920)
newcameramtx1, roi1 = cv2.getOptimalNewCameraMatrix(mtx1, dist1, (w, h), 1, (w, h))
x1, y1, w1, h1 = roi1
mapx1, mapy1 = cv2.initUndistortRectifyMap(mtx1, dist1, None, newcameramtx1, (w, h), 5)
ajust = 250

print(cv2.__version__)


def talker():
	rospy.init_node('image_processor', anonymous=True)
	image_cam_pub = rospy.Publisher("cam1", Image, queue_size=1)
	cap = cv2.VideoCapture(0)
	cap.set(3, 1920)
	cap.set(4, 1080)
	bridge = CvBridge()
	while not rospy.is_shutdown():
		try:
			ret, frame = cap.read()
			if ret:
				h = std_msgs.msg.Header()
				h.stamp = rospy.Time.now()
				image1undist  = cv2.remap(frame , mapx1, mapy1, cv2.INTER_LINEAR)
				image1undist  = image1undist [:, x1 - ajust: x1 + w1 + ajust]
				ros_image = bridge.cv2_to_imgmsg(image1undist,"bgr8")
				ros_image.header = h
				image_cam_pub.publish(ros_image)
		except Exception as e:
			print(e)


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		cap.release()
		pass

