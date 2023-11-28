#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import ros_numpy

def image_callback(msg):
    # Convert ROS Image message to NumPy array
    np_img = ros_numpy.numpify(msg)

    # Save the image
    cv2.imwrite('/home/noe/catkin_ws/src/Pami/src/EmptyImage.png', np_img)

    # Shutdown the node
    rospy.signal_shutdown("Image captured and saved.")

def main():
    rospy.init_node('image_listener', anonymous=True)
    
    # Subscribe to the /terrain topic
    rospy.Subscriber("/terrain", Image, image_callback)

    # Keep python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()

