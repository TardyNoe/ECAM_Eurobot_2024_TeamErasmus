#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

def create_binary_image():
    height, width = 600, 400
    image = np.zeros((height, width), dtype=np.uint8)
    center = (width // 2, height // 2)
    radius = min(height, width) // 4
    cv2.circle(image, center, radius, (255, 255, 255), -1)
    return image

def main():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('/mask', Image, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        binary_image = create_binary_image()
        ros_image = bridge.cv2_to_imgmsg(binary_image, encoding="mono8")
        pub.publish(ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
