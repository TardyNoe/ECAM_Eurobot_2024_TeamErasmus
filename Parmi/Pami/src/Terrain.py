#!/usr/bin/env python3
import ImgObjPoints
import Straightener
import rospy
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher:
    def __init__(self):
        self.cam1_sub = rospy.Subscriber("cam1", Image, self.callback_cam1)
        self.terrain_pub = rospy.Publisher("/terrain", Image, queue_size=10)
        self.cam1_image = None
        self.st = Straightener.Straightener()
        self.correspondance = None
        self.bridge = CvBridge()

    def callback_cam1(self, data):
        self.cam1_image =  self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        self.process_image()

    def publish_terrain(self, terrain):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(terrain, "bgr8")
            self.terrain_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr("Error converting terrain image: %s", e)

    def process_image(self):
        if self.cam1_image is not None and self.correspondance is not None:
            terrain = self.st.computeImage(self.cam1_image, self.correspondance)
            self.publish_terrain(terrain)

    def calibrate(self, ref_image, real_height):
        try:
            obj = ImgObjPoints.ObjImg(ref_image, real_height)
            
            img_points, obj_points = obj.getImgpointsAndObjpoints(self.cam1_image)
            print(obj.detectedtags)
            if obj.detectedtags != 4:
                return
            self.correspondance = self.st.findCorrespondance(img_points, obj_points)
            print("Calibrated ! with "+str(obj.detectedtags)+" tags")
            
        except Exception as e:
            print(e)

def main():
    rospy.init_node('image_subscriber_node', anonymous=True)
    img_subscriber = ImagePublisher()

    real_height = 3
    ref_image_path = "/home/noe/catkin_ws/src/Pami/src/table.png"
    ref_image = cv2.imread(ref_image_path)
    ref_image = cv2.resize(ref_image, (800, 1200), interpolation=cv2.INTER_AREA)
    rospy.sleep(1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if img_subscriber.correspondance is None:
            img_subscriber.calibrate(ref_image, real_height)
        rate.sleep()

if __name__ == '__main__':
    main()

