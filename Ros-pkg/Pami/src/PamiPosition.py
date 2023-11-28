#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

trsh = 60
emptyImage = None
kernel = np.ones((5, 5), np.uint8)  # Assuming a kernel for erosion and dilation

lower = np.array([32, 30, 150])
upper = np.array([90, 255, 255])

def image_callback(msg):
    # Convert ROS Image message to NumPy array
    np_image = ros_numpy.numpify(msg)
    hsv_img = cv2.cvtColor(np_image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    erode_mask = cv2.erode(mask, kernel, iterations=3)
    dilated_mask = cv2.dilate(erode_mask, kernel, iterations=4)
    masked_image = cv2.bitwise_and(np_image, np_image, mask=dilated_mask)
    circles = cv2.HoughCircles(dilated_mask, cv2.HOUGH_GRADIENT, dp=6, minDist=20, param1=100, param2=30, minRadius=10, maxRadius=20)
    
    if circles is not None:
        x, y, _ = circles[0][0]  # Assuming you want the first circle's center
        z = 0

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "map"  # Replace with your frame_id
        pose.pose.position.x = -(x-400)/200
        pose.pose.position.y = (y-300)/200
        pose.pose.position.z = z

        pose_pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    
    # Create a subscriber to the '/terrain' topic
    rospy.Subscriber('/terrain', Image, image_callback)

    # Create a publisher for PoseStamped messages on the '/Pami' topic
    pose_pub = rospy.Publisher('/Pami', PoseStamped, queue_size=10)

    rospy.spin()

