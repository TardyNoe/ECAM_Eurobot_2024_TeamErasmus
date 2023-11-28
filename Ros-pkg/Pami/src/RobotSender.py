#!/usr/bin/env python
import socket
import rospy
from geometry_msgs.msg import Point,PointStamped,PoseStamped
from std_msgs.msg import Header
import math
import struct
import random
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool
esp_ip = "10.42.0.202"
esp_port = 1234
#local_ip = "10.42.0.227" 
local_ip = "10.42.0.1" 
local_port = 1234 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((local_ip, local_port))

angleGoal = int(180)
x,y = 0,0
xgoal,ygoal = 0,0
power = 0
state = 0;
tirette_msg = Bool()
color_msg = Bool()
def sendToRobot(angleGoal,power):	
    byte_array = angleGoal.to_bytes(2, byteorder='big')
    message = byte_array + int(min(1500*power,100)).to_bytes(1,byteorder='big')
    #message = byte_array + int(200).to_bytes(1,byteorder='big')
    sock.sendto(message, (esp_ip, esp_port))

def exctractData(data):
    try:
        x,y,theta,distance,state = struct.unpack('hhhbb',data)
        angle = -math.radians(theta/10)
        posx = (x/1000)*0.28
        posy = (-y/1000)*0.28
        Tirette = 0
        Color = 0
        if state == 0:
            Tirette = 0
            Color = 0
        elif state == 1:
            Tirette = 0
            Color = 1
        elif state == 2:
            Tirette = 1
            Color = 0
        elif state == 3:
            Tirette = 1
            Color = 1
        color_msg.data = Color
        tirette_msg.data = Tirette
        color_publisher.publish(color_msg)
        tirette_publisher.publish(tirette_msg)
        return posx,posy,angle,distance,Tirette,Color
        
    except:
        print("error data in")
        return 0,0,0,0
        
def publishPos(posx, posy, angle,pose_publisher):
    current_time = rospy.Time.now()
    pose_msg = PoseStamped()
    pose_msg.header.stamp = current_time
    pose_msg.header.frame_id = "map"
    pose_msg.pose.position.x = posx
    pose_msg.pose.position.y = posy
    pose_msg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, angle)
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]
    pose_publisher.publish(pose_msg)
    
def publishDistance(distance,posx, posy, angle,point_publisher):
    header = Header(stamp=rospy.Time.now(), frame_id="map")
    point_msg = Point(posx + (distance/100)*math.cos(angle), posy+ (distance/100)*math.sin(angle), 0)
    my_point_stamped = PointStamped(header=header, point=point_msg)
    if distance > 0:
        point_publisher.publish(my_point_stamped)

def computeCommand(posx,posy,xgoal,ygoal,angleGoal,angle):
    power = 3*math.sqrt((posx-xgoal)**2+(posy-ygoal)**2)	
    angleGoal = int(math.degrees(math.atan2((ygoal-posy),(xgoal-posx))) +180)
    if math.sqrt((posx-xgoal)**2+(posy-ygoal)**2)<0.01:
        power = 0
    if tirette_msg.data == 0:
        angleGoal = 180
        power = 0
    if xgoal == 0 and ygoal == 0:
        angleGoal = 180
        power = 0
    #if abs(angleGoal+ math.degrees(angle))>15:
    #    power = 0
    return power,angleGoal

def callback(msg):
    global angleGoal
    global xgoal,ygoal
    orientation_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)
    xgoal,ygoal = msg.pose.position.x, msg.pose.position.y
    yaw_degrees = yaw * 180.0 / 3.14159265358979323846
    #print(yaw_degrees)

if __name__ == '__main__':
    rospy.init_node('tf_publisher_node')
    point_publisher = rospy.Publisher('point_topic', PointStamped, queue_size=10)
    pose_publisher = rospy.Publisher('/Pami/LocalPosition', PoseStamped, queue_size=10)
    tirette_publisher = rospy.Publisher('tirette', Bool, queue_size=10)
    color_publisher = rospy.Publisher('color', Bool, queue_size=10)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    x,y,theta,distance = 0,0,0,0
    power,angleGoal = 0,0
    rate = rospy.Rate(50)  # 60Hz rate
    while not rospy.is_shutdown():
    
        #data = ser.read(7)
        sendToRobot(angleGoal,power)
        try:
            sock.settimeout(0.1)
            data,addr = sock.recvfrom(1024)
            posx,posy,angle,distance,Tirette,Color = exctractData(data)	
            print(Tirette,Color)
            publishPos(posx, posy, angle,pose_publisher)
            publishDistance(distance,posx, posy, angle,point_publisher)
            power,angleGoal = computeCommand(posx,posy,xgoal,ygoal,angleGoal,angle)
            #print("sent")
        except Exception as e:
            #print(e)
            continue
        rate.sleep()
#
#



