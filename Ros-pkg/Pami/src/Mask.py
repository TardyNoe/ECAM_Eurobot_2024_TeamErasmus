#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

trsh = 60
emptyImage = None
kernel = np.ones((5,5), np.uint8)
robot_position = (0,0)

def position_callback(pose_stamped):
    """Callback function for the PoseStamped topic."""
    global robot_position
    robot_position = (pose_stamped.pose.position.x, pose_stamped.pose.position.y)

def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    np_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Apply a blur filter
    grayscaleRef = cv2.cvtColor(emptyImage, cv2.COLOR_BGR2GRAY)
    grayscaleTerrain = cv2.cvtColor(np_image, cv2.COLOR_BGR2GRAY)
    blurredTerrain = cv2.GaussianBlur(grayscaleRef, (9, 9), 2)
    blurredRef = cv2.GaussianBlur(grayscaleTerrain, (9, 9), 2)
    difference_image = cv2.absdiff(blurredTerrain, blurredRef)
    threshold_method = cv2.THRESH_BINARY
    _, result = cv2.threshold(difference_image, trsh, 255, threshold_method)
    result = cv2.dilate(result, kernel, iterations=2)
    result = cv2.resize(result, (40,60), interpolation=cv2.INTER_AREA)
    _, result = cv2.threshold(result, trsh, 255, threshold_method)
    posrobot = (int(robot_position[1]*20+30),int(-robot_position[0]*20+40))
    result = cv2.circle(result, (posrobot[1],posrobot[0]),5, 0, -1)
    result = cv2.dilate(result, kernel, iterations=1)

    # Convert the processed OpenCV image back to a ROS Image message
    image_message = bridge.cv2_to_imgmsg(result, encoding='mono8')
    pub.publish(image_message)

if __name__ == '__main__':
    emptyImage = cv2.imread("/home/noe/catkin_ws/src/Pami/src/EmptyImage.png")
    rospy.init_node('image_processor', anonymous=True)

    bridge = CvBridge()

    rospy.Subscriber('/terrain', Image, image_callback)
    rospy.Subscriber('/Pami', PoseStamped, position_callback)

    pub = rospy.Publisher('/mask', Image, queue_size=10)

    rospy.spin()

