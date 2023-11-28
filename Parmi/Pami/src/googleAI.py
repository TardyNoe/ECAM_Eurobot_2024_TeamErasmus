#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image
import tensorflow as tf
import time

# Charger le modèle TensorFlow Lite
tflite_model_path = '/home/noe/catkin_ws/src/Pami/src/model.tflite' # Mettez ici le chemin vers votre fichier .tflite
interpreter = tf.lite.Interpreter(model_path=tflite_model_path)
interpreter.allocate_tensors()

# Obtenir les détails des entrées et sorties
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()



def image_callback(msg):
    # Convert ROS Image message to NumPy array
    np_image = ros_numpy.numpify(msg)
    img = cv2.resize(np_image, (512,512), interpolation = cv2.INTER_AREA)
    img_array = [img]
    interpreter.set_tensor(input_details[0]['index'], img_array)
    interpreter.invoke()
    boxes = interpreter.get_tensor(output_details[0]['index'])
    classes = interpreter.get_tensor(output_details[1]['index'])
    scores = interpreter.get_tensor(output_details[2]['index'])
    threshold = 0.3
    image = img_array[0]
    for box, cls, score in zip(boxes[0], classes[0], scores[0]):
        if score > threshold:    
            y1, x1, y2, x2 = box
            coord1 = (int(x1*512),int(y1*512))
            coord2 = (int(x2*512),int(y2*512))
            color = (50*cls,50*cls,255-50*cls)
            image = cv2.rectangle(image, coord1, coord2, color, 3) 
            print(color)
            
    image = cv2.resize(image, (400,600), interpolation = cv2.INTER_AREA)
    image_message = ros_numpy.msgify(Image, image, encoding='bgr8')
    pub.publish(image_message)

if __name__ == '__main__':
    emptyImage = cv2.imread("/home/noe/catkin_ws/src/Pami/src/EmptyImage.png")
    rospy.init_node('image_processor', anonymous=True)
    
    # Create a subscriber to the '/terrain' topic
    rospy.Subscriber('/terrain', Image, image_callback)

    # Create a publisher to the '/mask' topic
    pub = rospy.Publisher('/AI', Image, queue_size=10)
    # Spin to keep the script from exiting
    rospy.spin()

