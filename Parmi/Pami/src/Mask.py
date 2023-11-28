#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image
trsh = 60
emptyImage = None
kernel = np.ones((5,5), np.uint8)
def image_callback(msg):
    # Convert ROS Image message to NumPy array
    np_image = ros_numpy.numpify(msg)
    
    # Apply a blur filter
    grayscaleRef = cv2.cvtColor(emptyImage, cv2.COLOR_BGR2GRAY)
    grayscaleTerrain = cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY)
    blurredTerrain= cv2.GaussianBlur(grayscaleRef, (9, 9), 2)
    blurredRef = cv2.GaussianBlur(grayscaleTerrain, (9, 9), 2)
    difference_image = cv2.absdiff(blurredTerrain, blurredRef)
    threshold_method = cv2.THRESH_BINARY                
    _, result = cv2.threshold(difference_image, trsh, 255, threshold_method)
    result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel, iterations = 1)
    # Show the image
    cv2.imshow("Blurred Image", difference_image)
    cv2.waitKey(3)

    # Convert back to ROS Image message and publish
    image_message = ros_numpy.msgify(Image, result, encoding='mono8')
    pub.publish(image_message)

if __name__ == '__main__':
    emptyImage = cv2.imread("/home/noe/catkin_ws/src/Pami/src/EmptyImage.png")
    rospy.init_node('image_processor', anonymous=True)
    
    # Create a subscriber to the '/terrain' topic
    rospy.Subscriber('/terrain', Image, image_callback)

    # Create a publisher to the '/mask' topic
    pub = rospy.Publisher('/mask', Image, queue_size=10)
    # Spin to keep the script from exiting
    rospy.spin()

